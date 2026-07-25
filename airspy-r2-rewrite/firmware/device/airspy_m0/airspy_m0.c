/*
 * Copyright 2012 Jared Boone
 * Copyright 2013-2015 Benjamin Vernoux <bvernoux@airspy.com>
 * Copyright 2015 Ian Gilmour <ian@sdrsharp.com>
 *
 * This file is part of AirSpy (based on HackRF project).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <libopencm3/lpc43xx/cgu.h>
#include <libopencm3/lpc43xx/i2c.h>
#include <libopencm3/lpc43xx/gpio.h>
#include <libopencm3/lpc43xx/m0/nvic.h>
#include <libopencm3/lpc43xx/creg.h>
#include <libopencm3/lpc43xx/rgu.h>
#include <libopencm3/cm3/systick.h>

#include <airspy_core.h>
#include <si5351c.h>
#include <r820t.h>
#include <w25q80bv.h>
#include <rom_iap.h>
#include <signal_mcu.h>
#include <stream_contract.h>

#include "usb.h"
#include "usb_standard_request.h"

#include "usb_device.h"
#include "usb_endpoint.h"
#include "usb_descriptor.h"

#include "airspy_conf.h"
#include "airspy_usb_req.h"
#include "airspy_commands.h"
#include "airspy_rx.h"
#include "r820t.h"
#include "airspy_m0.hdr"

extern uint32_t cm4_data_share; /* defined in linker script */
extern uint32_t cm0_data_share; /* defined in linker script */

volatile unsigned int phase = 0;

volatile uint32_t *usb_bulk_buffer_offset = (&cm4_data_share);
volatile uint32_t *usb_bulk_buffer_length = ((&cm4_data_share)+1);
volatile uint32_t *last_offset_m0 = ((&cm4_data_share)+2);
static volatile airspy_stream_contract_t* const stream_contract =
  (airspy_stream_contract_t*)&cm4_data_share;

volatile airspy_mcore_t *start_adchs = (airspy_mcore_t *)(&cm0_data_share);
volatile airspy_mcore_t *set_samplerate = (airspy_mcore_t *)((&cm0_data_share)+1);
volatile airspy_mcore_t *set_packing = (airspy_mcore_t *)((&cm0_data_share)+2);

#define get_usb_buffer_offset() (usb_bulk_buffer_offset[0])
#define get_usb_buffer_length() (usb_bulk_buffer_length[0])

#define MASTER_TXEV_FLAG  ((uint32_t *) 0x40043130)
#define MASTER_TXEV_QUIT()  { *MASTER_TXEV_FLAG = 0x0; }

uint8_t* const usb_bulk_buffer = (uint8_t*)0x20004000;

const char version_string[] = " " AIRSPY_FW_GIT_TAG " " AIRSPY_FW_CHECKIN_DATE;

#ifdef AIRSPY_STREAM_WORK_TELEMETRY
static uint32_t m0_cycle_elapsed(const uint32_t started)
{
  return (started - STK_CVR) & STK_CVR_CURRENT;
}
#endif

static uint32_t adc_stream_failed_recovery_generation;

static void adc_stream_publish_retirement(
  volatile airspy_stream_buffer_record_t* const record)
{
  const uint32_t index =
    (uint32_t)(record - &stream_contract->buffers[0]);
  const uint32_t write =
    stream_contract->retire_queue_write_sequence;
  const uint32_t read =
    stream_contract->retire_queue_read_sequence;
  if (write - read >= AIRSPY_STREAM_RETIRE_QUEUE_COUNT)
  {
    /*
     * M4 detects this counter and reconstructs its masks from the records.
     * Never overwrite an unread queue entry and silently alias ownership.
     */
    stream_contract->retire_queue_overflows++;
    signal_sev();
    return;
  }

  stream_contract->retire_queue_entries[
    write % AIRSPY_STREAM_RETIRE_QUEUE_COUNT] = index;
  airspy_stream_publish_barrier();
  stream_contract->retire_queue_write_sequence = write + 1u;
  signal_sev();
}

enum {
  /*
   * SysTick is a 24-bit AHB-clock down-counter. Eight million cycles is
   * approximately 39 ms at 204 MHz and 67 ms at 120 MHz, comfortably beyond
   * a healthy M4 command while remaining below one counter wrap.
   */
  MCORE_COMMAND_TIMEOUT_CYCLES = 8u * 1000u * 1000u
};

static void wait_for_mcore_ack(
  volatile airspy_mcore_t* const mailbox)
{
  const uint32_t started = STK_CVR;
  while (mailbox->raw != 0)
  {
    const uint32_t elapsed = (started - STK_CVR) & STK_CVR_CURRENT;
    if (elapsed >= MCORE_COMMAND_TIMEOUT_CYCLES)
    {
      /*
       * M0 cannot revoke an unknown M4/DMA state safely. A whole-core reset is
       * the last-resort path; ordinary DMA and USB faults recover in place.
       */
      cpu_reset();
    }
  }
  airspy_stream_publish_barrier();
}

static void adc_stream_recover(const uint32_t request_generation)
{
  if (request_generation == 0
    || stream_contract->recovery_acknowledged_generation
      == request_generation
    || adc_stream_failed_recovery_generation == request_generation)
  {
    return;
  }

  /*
   * One DMA bank is in flight when the channel faults. Count it plus every
   * completed bank that is still owned by USB. This is deliberately an
   * estimate: the host may already have accepted part of the active packet.
   */
  uint32_t dropped_buffers = 1;
  for (uint32_t index = 0; index < AIRSPY_STREAM_BUFFER_COUNT; ++index)
  {
    volatile airspy_stream_buffer_record_t* const record =
      &stream_contract->buffers[index];
    if (record->produced_generation != record->retired_generation
      || record->submitted_generation != record->retired_generation)
    {
      dropped_buffers++;
    }
  }

  /*
   * Stop only the bulk transport, not the receiver state or tuner. Cancelling
   * the queue retires each dTD exactly once and releases every bank. Resume
   * without resetting DATA0/DATA1 so the host's already-pending reads remain
   * valid after the short gap.
   */
  usb_endpoint_disable(&usb_endpoint_bulk_in);
  for (uint32_t index = 0; index < AIRSPY_STREAM_BUFFER_COUNT; ++index)
  {
    volatile airspy_stream_buffer_record_t* const record =
      &stream_contract->buffers[index];
    if (record->submitted_generation != record->retired_generation)
    {
      /*
       * A submitted bank must have received its cancellation callback before
       * the hardware flush returned. Anything else is an ownership invariant
       * failure, and restarting DMA could overwrite a bank still visible to
       * the USB engine.
       */
      stream_contract->dma_recovery_failure_count++;
      adc_stream_failed_recovery_generation = request_generation;
      return;
    }
    if (record->produced_generation != record->retired_generation)
    {
      /*
       * The producer can fault after publishing a bank but before M0 has
       * attached a dTD. No hardware owns that bank, so explicitly retire it
       * as a dropped buffer in this recovery epoch.
       */
      record->retired_bytes = 0;
      record->submitted_generation = record->produced_generation;
      airspy_stream_publish_barrier();
      record->retired_generation = record->produced_generation;
    }
  }
  usb_endpoint_resume(&usb_endpoint_bulk_in);

  stream_contract->dma_recovery_dropped_buffer_estimate += dropped_buffers;
  adc_stream_failed_recovery_generation = 0;
  airspy_stream_publish_barrier();
  stream_contract->recovery_acknowledged_generation = request_generation;
  signal_sev();
}

static void usb_suspend_changed(const bool suspended)
{
  airspy_stream_publish_barrier();
  if (suspended)
  {
    stream_contract->usb_suspended = 1;
    stream_contract->usb_suspend_count++;
    return;
  }

  if (stream_contract->usb_suspended == 0)
  {
    return;
  }
  /*
   * A full ring means suspension outlasted the available buffering. Banks
   * remain ordered and owned, but live RF time was necessarily lost.
   */
  if (stream_contract->capture_halted != 0)
  {
    stream_contract->usb_suspend_discontinuities++;
  }
  stream_contract->usb_suspended = 0;
  stream_contract->usb_resume_count++;
}

static void usb_bus_event(const uint32_t status)
{
  if (status & USB0_USBSTS_D_SEI)
  {
    /*
     * System Error means the controller halted itself after an AHB-master
     * fault. Only HCReset followed by full initialisation is valid. A whole
     * MCU reset takes that already-hardened boot path and safely terminates
     * every transport/capture owner.
     */
    stream_contract->usb_system_error_count++;
    airspy_stream_publish_barrier();
    cpu_reset();
  }
  if (status & USB0_USBSTS_D_UEI)
  {
    stream_contract->usb_controller_error_irq_count++;
  }
  if (status & USB0_USBSTS_D_URI)
  {
    stream_contract->usb_bus_reset_count++;
  }
  if (status & USB0_USBSTS_D_PCI)
  {
    stream_contract->usb_port_change_count++;
  }
}

static void publish_mcore_command(
  volatile airspy_mcore_t* const mailbox,
  const uint8_t conf,
  const uint8_t command)
{
  /*
   * The mailbox is naturally aligned and the LPC4370 performs an aligned
   * 32-bit SRAM store atomically. Publish conf and cmd together so M4 can
   * never observe a new command paired with the preceding configuration.
   */
  const uint32_t raw = ((uint32_t)conf << 8) | command;
  airspy_stream_publish_barrier();
  mailbox->raw = raw;
  airspy_stream_publish_barrier();
  signal_sev();
}

static void adc_stream_retired(
  void* const context,
  const uint32_t generation,
  const uint32_t actual_length,
  const usb_transfer_retirement_status_t status)
{
#ifdef AIRSPY_STREAM_WORK_TELEMETRY
  const uint32_t started = STK_CVR;
#endif
  volatile airspy_stream_buffer_record_t* const record = context;
  record->retired_bytes = actual_length;
  if (status == USB_TRANSFER_RETIREMENT_CANCELLED)
  {
    stream_contract->usb_cancelled++;
  }
  else if (status != USB_TRANSFER_RETIREMENT_COMPLETE)
  {
    stream_contract->usb_errors++;
    stream_contract->usb_queue_recovery_count++;
  }
  else if (actual_length != AIRSPY_STREAM_BUFFER_BYTES)
  {
    stream_contract->usb_partial++;
    stream_contract->usb_partial_discontinuity_count++;
  }
  if (record->submitted_generation != generation)
  {
    /*
     * A stale callback must never release a newer capture generation. Leave
     * the record owned so the M4 overwrite guard halts safely.
     */
    record->flags |= AIRSPY_STREAM_BUFFER_FLAG_OVERWRITE_RISK;
    stream_contract->usb_errors++;
    stream_contract->stale_generation_completions++;
    signal_sev();
    return;
  }
  airspy_stream_publish_barrier();
  record->retired_generation = generation;
  adc_stream_publish_retirement(record);
  stream_contract->usb_retired++;
#ifdef AIRSPY_STREAM_WORK_TELEMETRY
  const uint32_t elapsed = m0_cycle_elapsed(started);
  stream_contract->m0_retire_cycles_total += elapsed;
  if (elapsed > stream_contract->m0_retire_cycles_maximum)
  {
    stream_contract->m0_retire_cycles_maximum = elapsed;
  }
  stream_contract->m0_retire_count++;
#endif
  signal_sev();
}

static void adc_stream_submit_ready(void)
{
#ifdef AIRSPY_STREAM_WORK_TELEMETRY
  const uint32_t started = STK_CVR;
#endif
  uint32_t submitted = 0;
  const uint32_t write =
    stream_contract->grant_queue_write_sequence;
  uint32_t read =
    stream_contract->grant_queue_read_sequence;
  airspy_stream_publish_barrier();

  while (read != write)
  {
    const uint32_t index = stream_contract->grant_queue_entries[
      read & (AIRSPY_STREAM_GRANT_QUEUE_COUNT - 1u)];
    if (index >= AIRSPY_STREAM_BUFFER_COUNT)
    {
      stream_contract->usb_errors++;
      cpu_reset();
    }

    volatile airspy_stream_buffer_record_t* const record =
      &stream_contract->buffers[index];
    const uint32_t generation = record->granted_generation;
    if (generation == 0
      || generation != record->produced_generation
      || generation == record->submitted_generation)
    {
      stream_contract->usb_errors++;
      cpu_reset();
    }

    airspy_stream_publish_barrier();
    record->submitted_generation = generation;
    if (usb_transfer_schedule_tagged(
          &usb_endpoint_bulk_in,
          (void*)record->address,
          AIRSPY_STREAM_BUFFER_BYTES,
          adc_stream_retired,
          (void*)record,
          generation) != 0)
    {
      record->submitted_generation = record->retired_generation;
      stream_contract->usb_backpressure++;
      return;
    }
    stream_contract->usb_submitted++;
    submitted++;
    read++;
    airspy_stream_publish_barrier();
    stream_contract->grant_queue_read_sequence = read;
  }
#ifdef AIRSPY_STREAM_WORK_TELEMETRY
  if (submitted != 0)
  {
    const uint32_t elapsed = m0_cycle_elapsed(started);
    stream_contract->m0_submit_cycles_total += elapsed;
    if (elapsed > stream_contract->m0_submit_cycles_maximum)
    {
      stream_contract->m0_submit_cycles_maximum = elapsed;
    }
    stream_contract->m0_submit_count++;
  }
#else
  (void)submitted;
#endif
}

bool airspy_stream_requires_restart(void)
{
  return stream_contract->mode == AIRSPY_STREAM_MODE_POISONED;
}

static void adc_stream_terminate_poisoned_epoch(void)
{
  if (stream_contract->poison_transport_terminated != 0)
  {
    return;
  }

  /*
   * Cancel every data-bearing dTD, preserve the endpoint toggle, then answer
   * one pending host read with a ZLP. Stock libairspy treats that short bulk
   * transfer as fatal, which is the required epoch boundary.
   */
  usb_endpoint_disable(&usb_endpoint_bulk_in);
  usb_endpoint_resume(&usb_endpoint_bulk_in);
  if (usb_transfer_schedule_ack(&usb_endpoint_bulk_in) != 0)
  {
    stream_contract->usb_errors++;
    cpu_reset();
  }
  airspy_stream_publish_barrier();
  stream_contract->poison_transport_terminated = 1;
}

__attribute__ ((always_inline)) static inline void start_stop_adchs_m4(uint8_t conf_num, uint8_t command)
{
  publish_mcore_command(start_adchs, conf_num, command);
  wait_for_mcore_ack(start_adchs);
}

void set_samplerate_m4(uint8_t conf_num)
{
  publish_mcore_command(set_samplerate, conf_num, SET_SAMPLERATE_CMD);
  wait_for_mcore_ack(set_samplerate);
}

void set_packing_m4(uint8_t state)
{
  publish_mcore_command(set_packing, state, SET_PACKING_CMD);
  wait_for_mcore_ack(set_packing);
}

void usb_configuration_changed(usb_device_t* const device)
{
  if( device->configuration->number )
  {
    const receiver_mode_t desired_mode = get_receiver_mode();

    /*
     * SET_CONFIGURATION is a USB data-toggle reset boundary. Configure
     * the bulk endpoint once with DATA0, then application-level receiver
     * starts can preserve that shared host/device toggle state.
     */
    set_receiver_mode(RECEIVER_MODE_OFF);
    usb_endpoint_init(&usb_endpoint_bulk_in);
    usb_endpoint_disable(&usb_endpoint_bulk_in);
    if (desired_mode != RECEIVER_MODE_OFF)
    {
      set_receiver_mode(desired_mode);
    }
  } else
  {
    /* RECEIVER OFF */
    /* Configuration number equal 0 means usb bus reset. */
    set_receiver_mode(RECEIVER_MODE_OFF);
  }
}

void ADCHS_prepare(uint8_t conf_num)
{
  start_stop_adchs_m4(conf_num, PREPARE_ADCHS_CMD);

  /* Complete all clock-dependent tuner work before the ADC can run. */
  i2c0_init(airspy_conf->i2c_conf.i2c0_pll1_ls_hs_conf_val); /* Si5351C I2C peripheral */
  i2c1_init(airspy_conf->i2c_conf.i2c1_pll1_hs_conf_val); /* R820T I2C peripheral */

  if((conf_num & AIRSPY_SAMPLERATE_CONF_ALT) == AIRSPY_SAMPLERATE_CONF_ALT)
  {
    conf_num = conf_num & (~AIRSPY_SAMPLERATE_CONF_ALT);
    r820t_init(&airspy_conf->r820t_conf_rw, airspy_conf->airspy_m0_m4_alt_conf[conf_num].airspy_m0_conf.r820t_if_freq);
    r820t_set_if_bandwidth(&airspy_conf->r820t_conf_rw, airspy_conf->airspy_m0_m4_alt_conf[conf_num].airspy_m0_conf.r820t_if_bw);
  }else
  {
    r820t_init(&airspy_conf->r820t_conf_rw, airspy_conf->airspy_m0_m4_conf[conf_num].airspy_m0_conf.r820t_if_freq);
    r820t_set_if_bandwidth(&airspy_conf->r820t_conf_rw, airspy_conf->airspy_m0_m4_conf[conf_num].airspy_m0_conf.r820t_if_bw);
  }
  phase = 1;
}

void ADCHS_start(uint8_t conf_num)
{
  start_stop_adchs_m4(conf_num, START_ADCHS_CMD);
}

void ADCHS_stop(uint8_t conf_num)
{
  r820t_standby();
  start_stop_adchs_m4(conf_num, STOP_ADCHS_CMD);
}

void PLL1_park(uint8_t conf_num)
{
  start_stop_adchs_m4(conf_num, PARK_PLL1_CMD);
  /* Re-Init I2C0 & I2C1 after PLL1 frequency is modified */
  i2c0_init(airspy_conf->i2c_conf.i2c0_pll1_ls_hs_conf_val); /* Si5351C I2C peripheral */
  i2c1_init(airspy_conf->i2c_conf.i2c1_pll1_ls_conf_val); /* R820T I2C peripheral */
}

/***************************/
/* adchs_isr managed by M4 */
/***************************/
void m4core_isr(void)
{
  MASTER_TXEV_QUIT();
}

/*
M0 Core Manage USB
*/
int main(void)
{
  iap_cmd_res_t iap_cmd_res;
  usb_descriptor_serial_number_t serial_number;
  airspy_usb_req_init();

  /*
   * USB reset and queue-state waits use this free-running down-counter for
   * bounded last-resort deadlines, so it must be live before USB is touched.
   */
  STK_RVR = STK_RVR_RELOAD;
  STK_CVR = 0;
  STK_CSR = STK_CSR_CLKSOURCE_AHB | STK_CSR_ENABLE;

  /* R820T Startup */
  r820t_startup(&airspy_conf->r820t_conf_rw);

  usb_set_configuration_changed_cb(usb_configuration_changed);
  usb_set_suspend_changed_cb(usb_suspend_changed);
  usb_set_fatal_error_cb(cpu_reset);
  usb_set_bus_event_cb(usb_bus_event);
  usb_peripheral_reset();

  usb_device_init(0, &usb_device);

  usb_queue_init(&usb_endpoint_control_out_queue);
  usb_queue_init(&usb_endpoint_control_in_queue);
  usb_queue_init(&usb_endpoint_bulk_out_queue);
  usb_queue_init(&usb_endpoint_bulk_in_queue);

  usb_endpoint_init(&usb_endpoint_control_out);
  usb_endpoint_init(&usb_endpoint_control_in);

  /* Read IAP Serial Number Identification */
  iap_cmd_res.cmd_param.command_code = IAP_CMD_READ_SERIAL_NO;
  iap_cmd_call(&iap_cmd_res);
  if(iap_cmd_res.status_res.status_ret == CMD_SUCCESS)
  {
    /* Only retrieve 2 last 32bits for Serial Number */
    serial_number.sn_32b[0] = iap_cmd_res.status_res.iap_result[2];
    serial_number.sn_32b[1] = iap_cmd_res.status_res.iap_result[3];
    usb_descriptor_fill_string_serial_number(serial_number);
  }

  nvic_set_priority(NVIC_USB0_IRQ, 255);

  nvic_set_priority(NVIC_M4CORE_IRQ, 1);
  nvic_enable_irq(NVIC_M4CORE_IRQ);

  usb_run(&usb_device);

  while(true)
  {
    signal_wfe();

    const uint32_t stream_mode = stream_contract->mode;
    if (stream_mode == AIRSPY_STREAM_MODE_POISONED)
    {
      adc_stream_terminate_poisoned_epoch();
      continue;
    }
    if (stream_mode == AIRSPY_STREAM_MODE_RECOVERING)
    {
      /*
       * M4 has halted channel 0. Retire transport ownership and acknowledge
       * a fresh epoch; M4 then rebuilds ADC/DMA without changing RX state.
       */
      airspy_stream_publish_barrier();
      adc_stream_recover(stream_contract->recovery_request_generation);
      continue;
    }

    if (stream_mode == AIRSPY_STREAM_MODE_ADC_FOUR_BUFFER)
    {
      adc_stream_submit_ready();
      continue;
    }

    if (stream_mode == AIRSPY_STREAM_MODE_LEGACY)
    {
      uint32_t offset = get_usb_buffer_offset();
      uint32_t length = get_usb_buffer_length();

      if(offset != *last_offset_m0)
      {
        usb_transfer_schedule_block(&usb_endpoint_bulk_in, &usb_bulk_buffer[offset], length);
        *last_offset_m0 = offset;
      }
    }
  }
}
