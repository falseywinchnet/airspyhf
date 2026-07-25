/*
 * Copyright 2013-2016 Benjamin Vernoux <bvernoux@airspy.com>
 * Copyright 2015 Ian Gilmour <ian@sdrsharp.com>
 *
 * This file is part of AirSpy.
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

#include <string.h>

#include <libopencm3/lpc43xx/cgu.h>
#include <libopencm3/lpc43xx/ccu.h>
#include <libopencm3/lpc43xx/gpio.h>
#include <libopencm3/lpc43xx/m4/nvic.h>
#include <libopencm3/lpc43xx/creg.h>
#include <libopencm3/lpc43xx/rgu.h>
#include <libopencm3/lpc43xx/ipc.h>
#include <libopencm3/cm3/scs.h>

#include <airspy_core.h>
#include <si5351c.h>
#include <w25q80bv.h>
#include <rom_iap.h>
#include <signal_mcu.h>
#include <stream_contract.h>

#include "adchs.h"

#include "m0_bin.h"
#include "m0s_bin.h"

#include "airspy_conf.h"

#define DEFAULT_ADCHS_CHAN (0)

#undef DMA_ISR_DEBUG
//#define DMA_ISR_DEBUG

#define USB_DATA_TRANSFER_SIZE_BYTE (ADCHS_DATA_TRANSFER_SIZE_BYTE)
#define USB_BULK_BUFFER_MASK ((32768) - 1)
#define get_usb_buffer_offset() (usb_bulk_buffer_offset[0])
#define set_usb_buffer_offset(val) (usb_bulk_buffer_offset[0] = val)
/* Manage round robin after increment with USB_BULK_BUFFER_MASK */
#define inc_mask_usb_buffer_offset(buff_offset, inc_value) ((buff_offset+inc_value) & USB_BULK_BUFFER_MASK)

volatile uint32_t usb_bulk_buffer_offset_uint32_m4;
volatile uint32_t *usb_bulk_buffer_offset_m4;
volatile uint32_t last_offset_m4;
#define get_usb_buffer_offset_m4() (usb_bulk_buffer_offset_m4[0])
#define set_usb_buffer_offset_m4(val) (usb_bulk_buffer_offset_m4[0] = val)
#define inc_mask_usb_buffer_offset_m4(buff_offset, inc_value) inc_mask_usb_buffer_offset(buff_offset, inc_value)

#define SLAVE_TXEV_FLAG ((uint32_t *) 0x40043400)
#define SLAVE_TXEV_QUIT() { *SLAVE_TXEV_FLAG = 0x0; }

extern uint32_t cm4_data_share; /* defined in linker script */
extern uint32_t adchs_data; /* defined in linker script */
extern uint32_t cm0_data_share; /* defined in linker script */

volatile int adchs_stopped = 0;
volatile int adchs_started = 0;
static volatile uint8_t adchs_prepared = 0;
static volatile uint8_t pll1_is_parked = 1;

volatile int use_packing = 0;

volatile uint32_t *usb_bulk_buffer_offset = &cm4_data_share;
volatile uint32_t *usb_bulk_buffer_length = ((&cm4_data_share)+1);
volatile uint32_t *last_offset_m0 = ((&cm4_data_share)+2);
volatile airspy_stream_contract_t* const stream_contract =
  (airspy_stream_contract_t*)&cm4_data_share;

uint8_t* const usb_bulk_buffer = (uint8_t*)USB_BULK_BUFFER_START;

volatile airspy_mcore_t *start_adchs = (airspy_mcore_t *)(&cm0_data_share);
volatile airspy_mcore_t *set_samplerate = (airspy_mcore_t *)((&cm0_data_share)+1);
volatile airspy_mcore_t *set_packing = (airspy_mcore_t *)((&cm0_data_share)+2);

volatile int first_start = 0;
static uint32_t adc_ring_completed_index;
static uint32_t adc_ring_generation;
static uint32_t adc_ring_last_completion_cycles;
static uint32_t adc_ring_halted;
static uint32_t adc_ring_protected_index;

static uint32_t adc_ring_next_index(const uint32_t index)
{
  const uint32_t next = index + 1u;
  return next == AIRSPY_STREAM_BUFFER_COUNT ? 0u : next;
}

static void service_adc_ring_resume(void)
{
#ifdef AIRSPY_STREAM_WORK_TELEMETRY
  const uint32_t started = SCS_DWT_CYCCNT;
#endif
  if (adc_ring_halted == 0
    || stream_contract->mode == AIRSPY_STREAM_MODE_RECOVERING)
  {
    return;
  }

  volatile airspy_stream_buffer_record_t* const protected =
    &stream_contract->buffers[adc_ring_protected_index];
  if (protected->produced_generation != protected->retired_generation
    || protected->submitted_generation != protected->retired_generation)
  {
    return;
  }

  const uint32_t adc_status = LPC_ADCHS->STATUS0;
  if (adc_status & STAT0_FIFO_OVERFLOW)
  {
    stream_contract->adc_fifo_overflow_count++;
    stream_contract->backpressure_discontinuity_count++;
  }
  LPC_ADCHS->CLR_STAT0 = adc_status;
  protected->flags &= ~AIRSPY_STREAM_BUFFER_FLAG_OVERWRITE_RISK;
  airspy_stream_publish_barrier();
  LPC_GPDMA->C0CONFIG &= ~(1u << 18);
  adc_ring_halted = 0;
  stream_contract->capture_halted = 0;
#ifdef AIRSPY_STREAM_WORK_TELEMETRY
  const uint32_t elapsed = SCS_DWT_CYCCNT - started;
  stream_contract->m4_resume_cycles_total += elapsed;
  if (elapsed > stream_contract->m4_resume_cycles_maximum)
  {
    stream_contract->m4_resume_cycles_maximum = elapsed;
  }
  stream_contract->m4_resume_count++;
#endif
}

static uint32_t stream_checksum(
  const uint32_t* const data,
  const uint32_t length)
{
  uint32_t checksum = 0x811c9dc5u;
  for (uint32_t index = 0; index < length / sizeof(uint32_t); ++index)
  {
    checksum ^= data[index];
    checksum *= 0x01000193u;
  }
  return checksum;
}

static void stream_contract_init(void)
{
  volatile uint32_t* const words = (volatile uint32_t*)stream_contract;
  for (uint32_t index = 0;
    index < sizeof(*stream_contract) / sizeof(uint32_t); ++index)
  {
    words[index] = 0;
  }

  stream_contract->magic = AIRSPY_STREAM_CONTRACT_MAGIC;
  stream_contract->version = AIRSPY_STREAM_CONTRACT_VERSION;
  stream_contract->mode = AIRSPY_STREAM_MODE_LEGACY;
  stream_contract->buffer_count = AIRSPY_STREAM_BUFFER_COUNT;
  stream_contract->buffer_bytes = AIRSPY_STREAM_BUFFER_BYTES;
  stream_contract->minimum_completion_cycles = 0xffffffffu;
  /*
   * Alternate the five local-SRAM1 banks with the five destinations on other
   * SRAM slaves. The former order ended with two adjacent local-SRAM1 banks
   * and produced one deterministic ADC FIFO overflow per ten-bank revolution
   * at the R2's 20 MS/s raw capture rate.
   */
  stream_contract->buffers[0].address = 0x20004000u;
  stream_contract->buffers[1].address = 0x10008000u;
  stream_contract->buffers[2].address = 0x10084000u;
  stream_contract->buffers[3].address = 0x1000c000u;
  stream_contract->buffers[4].address = 0x20008000u;
  stream_contract->buffers[5].address = 0x10010000u;
  stream_contract->buffers[6].address = 0x10088000u;
  stream_contract->buffers[7].address = 0x10014000u;
  stream_contract->buffers[8].address = 0x1008c000u;
  stream_contract->buffers[9].address = 0x10018000u;
  airspy_stream_publish_barrier();
}

static void service_gpdma_probe(void)
{
  const uint32_t command = stream_contract->gpdma_command_generation;
  if (command == stream_contract->gpdma_completed_generation)
  {
    return;
  }

  airspy_stream_publish_barrier();
  const uint32_t destination_index =
    stream_contract->gpdma_destination_index;
  uint32_t cycles = 0;
  uint32_t error_status = 0;
  int result = -1;

  if (adchs_started == 0
    && destination_index >= 2
    && destination_index < AIRSPY_STREAM_BUFFER_COUNT)
  {
    result = GPDMA_memory_copy_probe(
      stream_contract->buffers[0].address,
      stream_contract->buffers[destination_index].address,
      AIRSPY_STREAM_BUFFER_BYTES,
      &cycles,
      &error_status);
  }

  stream_contract->gpdma_bytes = AIRSPY_STREAM_BUFFER_BYTES;
  stream_contract->gpdma_cycles = cycles;
  stream_contract->gpdma_error_status = error_status;
  stream_contract->gpdma_expected_checksum = stream_checksum(
    (const uint32_t*)stream_contract->buffers[0].address,
    AIRSPY_STREAM_BUFFER_BYTES);
  stream_contract->gpdma_actual_checksum =
    destination_index < AIRSPY_STREAM_BUFFER_COUNT
    ? stream_checksum(
        (const uint32_t*)stream_contract->buffers[destination_index].address,
        AIRSPY_STREAM_BUFFER_BYTES)
    : 0;

  if (result == 0
    && stream_contract->gpdma_expected_checksum
      == stream_contract->gpdma_actual_checksum)
  {
    stream_contract->gpdma_status = AIRSPY_STREAM_GPDMA_PASSED;
  }
  else if (result == -2)
  {
    stream_contract->gpdma_status = AIRSPY_STREAM_GPDMA_TIMEOUT;
  }
  else
  {
    stream_contract->gpdma_status = AIRSPY_STREAM_GPDMA_ERROR;
  }

  airspy_stream_publish_barrier();
  stream_contract->gpdma_completed_generation = command;
  signal_sev();
}

/*
uint32_t nb_cycles[5] = { 0 };
uint32_t data_counter = 0;
*/
#ifdef DMA_ISR_DEBUG
  #define DMA_IRQ_CYCLES_MAX (100)
  #define FREQ_DMA_IRQ_CYCLES_MAX (100)
  typedef struct
  {
    uint32_t adchs_fifo_ovf;
    uint32_t adchs_dscr_error;
    uint32_t adchs_adc_ovf;
    uint32_t adchs_adc_unf;
    uint32_t dma_err_cnt;

    uint32_t dma_irq_cycles[DMA_IRQ_CYCLES_MAX];
    uint32_t dma_irq_cycles_idx;

    uint32_t freq_dma_irq_cycles[FREQ_DMA_IRQ_CYCLES_MAX];
    uint32_t freq_dma_irq_cycles_idx;
  } t_stats_adchs;

  t_stats_adchs stat_adchs = { 0 };
#endif

/*
__attribute__ ((always_inline)) static void pack(uint16_t* input, uint32_t* output, uint32_t length)
{
  uint32_t i;

  for (i = 0; i < length; i += 8)
  {
    register uint32_t t2, t5;

    t2 = input[i+2];

    output[0] = ((uint32_t)(input[i] << 20)) | ((uint32_t)(input[i+1] << 8)) | (t2 >> 4);
    t5 = input[i+5];
    output[1] = ((uint32_t)(t2&0xf) << 28)| ((uint32_t)input[i+3] << 16) | (input[i+4] << 4) | (t5 >> 8);
    output[2] = ((uint32_t)(t5 & 0xff) << 24) | ((uint32_t)input[i+6]<<12) | ((uint32_t)input[i+7]);

    output += 3;
  }
}*/

__attribute__ ((always_inline)) static inline void pack(
  uint32_t* input,
  uint32_t* output,
  uint32_t length)
{
  register uint32_t *a0 asm("r0") = input;
  register uint32_t *a1 asm("r1") = output;
  register uint32_t a2 asm("r2") = length;

  asm volatile("1:\n\t"
         "ldm.w %0!, {r4, r5, r6, r7}\n\t"

         "lsr	r8, r4, #16\n\t"
         "ubfx	r3, r5, #4, #12\n\t"
         "orr	r8, r3, r8, lsl #8\n\t"
         "orr	r8, r8, r4, lsl #20\n\t"
         "lsrs	r3, r5, #16\n\t"
         "lsls	r5, r5, #28\n\t"
         "orr	r5, r5, r3, lsl #16\n\t"
         "orr	r5, r5, r6, lsr #24\n\t"
         "uxth	r9, r6\n\t"
         "orr	r9, r5, r9, lsl #4\n\t"
         "lsrs	r6, r6, #16\n\t"
         "uxth	r10, r7\n\t"
         "lsl	r10, r10, #12\n\t"
         "orr	r10, r10, r6, lsl #24\n\t"
         "orr	r10, r10, r7, lsr #16\n\t"

         "stm.w %1!, {r8, r9, r10}\n\t"

         "subs	%2, %2, #8\n\t"
         "bne 1b\n\t"
        : "+r"(a0), "+r"(a1), "+r"(a2)
        :: "memory", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10");

}

static __inline__ void clr_usb_buffer_offset(void)
{
  if(use_packing)
  {
    usb_bulk_buffer_offset[0] = ADCHS_DATA_TRANSFER_SIZE_BYTE / 2;
    usb_bulk_buffer_offset_m4[0] = ADCHS_DATA_TRANSFER_SIZE_BYTE;
  }
  else
  {
    usb_bulk_buffer_offset[0] = ADCHS_DATA_TRANSFER_SIZE_BYTE;
  }

  last_offset_m4 = 0;
  *last_offset_m0 = 0;
}

static __inline__ uint32_t get_start_stop_adchs(void)
{
  const uint32_t raw = start_adchs->raw;
  airspy_stream_publish_barrier();
  return raw & 0xffu;
}

/* Acknowledge Start/Stop ADCHS by clearing the data */
static __inline__ void ack_start_stop_adchs(void)
{
  airspy_stream_publish_barrier();
  start_adchs->raw  = 0;
  airspy_stream_publish_barrier();
}

static __inline__ uint8_t get_samplerate(uint8_t *conf_number)
{
  const uint32_t raw = set_samplerate->raw;
  airspy_stream_publish_barrier();
  *conf_number = (raw >> 8) & 0xffu;
  return raw & 0xffu;
}

/* Acknowledge set_samplerate by clearing the data */
static __inline__ void ack_samplerate(void)
{
  airspy_stream_publish_barrier();
  set_samplerate->raw = 0;
  airspy_stream_publish_barrier();
}

static __inline__ uint8_t get_packing(uint8_t *packing_state)
{
  const uint32_t raw = set_packing->raw;
  airspy_stream_publish_barrier();
  *packing_state = (raw >> 8) & 0xffu;
  return raw & 0xffu;
}

static __inline__ void ack_packing(void)
{
  airspy_stream_publish_barrier();
  set_packing->raw = 0;
  airspy_stream_publish_barrier();
}

void set_packing_state(uint8_t state)
{
  if(state == 0)
  {
    use_packing = 0;
    *usb_bulk_buffer_length = 0x4000;
  }
  else
  {
    use_packing = 1;
    *usb_bulk_buffer_length = 0x1800;
  }
}

static void adchs_prepare(void)
{
  cpu_clock_pll1_high_speed(&airspy_conf->airspy_m4_init_conf.pll1_hs);
  pll1_is_parked = 0;
  first_start = 1;
  stream_contract->clock_stream_pll1_ctrl = CGU_PLL1_CTRL;
  stream_contract->clock_base_m4 = CGU_BASE_M4_CLK;
  stream_contract->clock_base_periph = CGU_BASE_PERIPH_CLK;
  stream_contract->clock_base_apb1 = CGU_BASE_APB1_CLK;
  stream_contract->clock_base_apb3 = CGU_BASE_APB3_CLK;
  stream_contract->clock_high_transitions++;
  adchs_prepared = 1;
}

static void adchs_start_internal(const uint8_t chan_num)
{
  int i;
  uint32_t *dst;

  /* Disable IRQ globally */
  __asm__("cpsid i");

  clr_usb_buffer_offset();

  ADCHS_init();
  ADCHS_desc_init(chan_num);
  if (use_packing)
  {
    /* Preserve the original contiguous two-bank packing path. */
    dst = (uint32_t*)ADCHS_DATA_BUFFER;
    for (i = 0; i < ADCHS_DATA_BUFFER_SIZE_BYTE / 4; ++i)
    {
      dst[i] = 0;
    }
    stream_contract->mode = AIRSPY_STREAM_MODE_LEGACY;
    ADCHS_DMA_init((uint32_t)ADCHS_DATA_BUFFER, use_packing);
  }
  else
  {
    uint32_t destinations[AIRSPY_STREAM_BUFFER_COUNT];
    for (i = 0; i < AIRSPY_STREAM_BUFFER_COUNT; ++i)
    {
      volatile airspy_stream_buffer_record_t* const record =
        &stream_contract->buffers[i];
      destinations[i] = record->address;
      record->produced_generation = 0;
      record->submitted_generation = 0;
      record->retired_generation = 0;
      record->retired_bytes = 0;
      record->dma_start_cycles = 0;
      record->dma_complete_cycles = 0;
      record->flags = 0;
    }
    stream_contract->capture_generation = 0;
    stream_contract->capture_halted = 0;
    adc_ring_completed_index = 0;
    adc_ring_generation = 0;
    adc_ring_last_completion_cycles = SCS_DWT_CYCCNT;
    adc_ring_halted = 0;
    adc_ring_protected_index = 0;
    stream_contract->mode = AIRSPY_STREAM_MODE_ADC_FOUR_BUFFER;
    airspy_stream_publish_barrier();
    ADCHS_DMA_init_ring(destinations);
  }

  led_on();
  LPC_ADCHS->TRIGGER = 1;
  __asm("dsb");

  /* Enable IRQ globally */
  __asm__("cpsie i");
}

void adchs_start(uint8_t chan_num)
{
  adchs_start_internal(chan_num);
}

void adchs_stop(void)
{
  /* Disable IRQ globally */
  __asm__("cpsid i");

  ADCHS_deinit();
  stream_contract->mode = AIRSPY_STREAM_MODE_LEGACY;
  airspy_stream_publish_barrier();

  led_off();

  /* Enable IRQ globally */
  __asm__("cpsie i");
}

static void service_adc_recovery(void)
{
  if (stream_contract->mode != AIRSPY_STREAM_MODE_RECOVERING)
  {
    return;
  }

  const uint32_t request =
    stream_contract->recovery_request_generation;
  if (request == 0
    || stream_contract->recovery_acknowledged_generation != request)
  {
    return;
  }

  /*
   * M0 has cancelled every dTD owner and resumed an empty endpoint. Rebuild
   * ADC and channel 0 from a clean descriptor epoch without touching tuner,
   * sample-rate, gain, receiver mode, or accumulated diagnostics.
   */
  airspy_stream_publish_barrier();
  ADCHS_deinit();
  adchs_start_internal(DEFAULT_ADCHS_CHAN);
  stream_contract->dma_recovery_count++;
  airspy_stream_publish_barrier();
  stream_contract->recovery_completed_generation = request;
  signal_sev();
}

static void pll1_park(void)
{
  if (pll1_is_parked != 0)
  {
    return;
  }
  cpu_clock_pll1_low_speed(&airspy_conf->airspy_m4_init_conf.pll1_ls);
  pll1_is_parked = 1;
  stream_contract->clock_idle_pll1_ctrl = CGU_PLL1_CTRL;
  stream_contract->clock_low_transitions++;
}

void dma_isr(void)
{
#ifdef AIRSPY_STREAM_WORK_TELEMETRY
  const uint32_t isr_started = SCS_DWT_CYCCNT;
#endif
  uint32_t status;
  #define INTTC0  (1)
  const uint32_t error_status = LPC_GPDMA->INTERRSTAT;
  const uint32_t adc_status = LPC_ADCHS->STATUS0;

  if (adc_status != 0)
  {
    LPC_ADCHS->CLR_STAT0 = adc_status;
    if (adc_status & STAT0_FIFO_OVERFLOW)
    {
      stream_contract->adc_fifo_overflow_count++;
    }
    if (adc_status & STAT0_DSCR_ERROR)
    {
      stream_contract->adc_descriptor_error_count++;
    }
    if (adc_status & STAT0_ADC_OVF)
    {
      stream_contract->adc_overrange_count++;
    }
    if (adc_status & STAT0_ADC_UNF)
    {
      stream_contract->adc_underrange_count++;
    }
  }

  /*
   * This acknowledgement was debug-only in the upstream firmware. A single
   * production DMA error therefore left the interrupt asserted forever. Always
   * clear it. Channel 0 enters a recoverable epoch restart: M0 cancels USB
   * ownership, then M4 rebuilds ADC/DMA without changing receiver state.
   */
  if (error_status)
  {
    /* The IRQ is shared by all GPDMA channels; never leave any error set. */
    LPC_GPDMA->INTERRCLR = error_status;
  }
  if ((error_status & INTTC0)
    && stream_contract->mode != AIRSPY_STREAM_MODE_RECOVERING)
  {
    uint32_t request =
      stream_contract->recovery_request_generation + 1u;
    if (request == 0)
    {
      request = 1;
    }
    LPC_GPDMA->C0CONFIG |= 1u << 18;
    adc_ring_halted = 1;
    stream_contract->dma_error_count++;
    stream_contract->capture_halted = 1;
    stream_contract->last_dma_error_status = error_status;
    stream_contract->recovery_request_generation = request;
    airspy_stream_publish_barrier();
    stream_contract->mode = AIRSPY_STREAM_MODE_RECOVERING;
    airspy_stream_publish_barrier();
    signal_sev();
  }

#ifdef DMA_ISR_DEBUG
  volatile uint32_t tmp_cycles;
  tmp_cycles = SCS_DWT_CYCCNT;

  stat_adchs.freq_dma_irq_cycles[stat_adchs.freq_dma_irq_cycles_idx] = tmp_cycles;
  stat_adchs.freq_dma_irq_cycles_idx++;
  if(stat_adchs.freq_dma_irq_cycles_idx == FREQ_DMA_IRQ_CYCLES_MAX)
    stat_adchs.freq_dma_irq_cycles_idx = 0;

  // ADCHS Error stat0

  /* FIFO was full; conversion sample is not stored and lost */
  if(adc_status & STAT0_FIFO_OVERFLOW)
    stat_adchs.adchs_fifo_ovf++;

  /* The ADC was not fully woken up when a sample was
     converted and the conversion results is unreliable */
  if(adc_status & STAT0_DSCR_ERROR)
    stat_adchs.adchs_dscr_error++;

  /* Converted sample value was over range of the 12 bit output code. */
  if(adc_status & STAT0_ADC_OVF)
    stat_adchs.adchs_adc_ovf++;

  /* Converted sample value was under range of the 12 bit output code. */
  if(adc_status & STAT0_ADC_UNF)
    stat_adchs.adchs_adc_unf++;

  if (error_status)
    stat_adchs.dma_err_cnt++;
#endif

  status = LPC_GPDMA->INTTCSTAT;
  if (status & INTTC0)
  {
    LPC_GPDMA->INTTCCLEAR = INTTC0; /* Clear Chan0 */

    if (error_status & INTTC0)
    {
      /* Error takes precedence over a coincident terminal-count event. */
    }
    else if(use_packing)
    {
        set_usb_buffer_offset_m4( inc_mask_usb_buffer_offset_m4(get_usb_buffer_offset_m4(), 8192));
    }
    else
    {
      const uint32_t now = SCS_DWT_CYCCNT;
      const uint32_t elapsed = now - adc_ring_last_completion_cycles;
      volatile airspy_stream_buffer_record_t* const completed =
        &stream_contract->buffers[adc_ring_completed_index];
      uint32_t generation = adc_ring_generation + 1u;
      if (generation == 0)
      {
        generation = 1;
      }

      completed->dma_start_cycles = adc_ring_last_completion_cycles;
      completed->dma_complete_cycles = now;
      completed->flags = 0;
      if (completed->produced_generation != completed->retired_generation)
      {
        completed->flags |= AIRSPY_STREAM_BUFFER_FLAG_OVERWRITE_RISK;
        stream_contract->dma_error_count++;
        stream_contract->ownership_overwrite_count++;
      }
      airspy_stream_publish_barrier();
      completed->produced_generation = generation;
      stream_contract->capture_generation = generation;
      stream_contract->capture_completed++;
      if (elapsed < stream_contract->minimum_completion_cycles)
      {
        stream_contract->minimum_completion_cycles = elapsed;
      }
      if (elapsed > stream_contract->maximum_completion_cycles)
      {
        stream_contract->maximum_completion_cycles = elapsed;
      }

      /*
       * DMA is already entering the next bank. The bank after that must be
       * free now; otherwise halt after the in-flight work and never reach it.
       */
      const uint32_t protected_index =
        adc_ring_next_index(adc_ring_next_index(adc_ring_completed_index));
      volatile airspy_stream_buffer_record_t* const protected =
        &stream_contract->buffers[protected_index];
      if (protected->produced_generation != protected->retired_generation
        || protected->submitted_generation != protected->retired_generation)
      {
        protected->flags |= AIRSPY_STREAM_BUFFER_FLAG_OVERWRITE_RISK;
        stream_contract->overwrite_prevented++;
        stream_contract->capture_halted = 1;
        adc_ring_halted = 1;
        adc_ring_protected_index = protected_index;
        LPC_GPDMA->C0CONFIG |= 1u << 18;
      }

      adc_ring_generation = generation;
      adc_ring_completed_index =
        adc_ring_next_index(adc_ring_completed_index);
      adc_ring_last_completion_cycles = now;
      signal_sev();
    }
  }

#ifdef DMA_ISR_DEBUG
  stat_adchs.dma_irq_cycles[stat_adchs.dma_irq_cycles_idx] = (SCS_DWT_CYCCNT - tmp_cycles);
  stat_adchs.dma_irq_cycles_idx++;
  if(stat_adchs.dma_irq_cycles_idx == DMA_IRQ_CYCLES_MAX)
    stat_adchs.dma_irq_cycles_idx = 0;
#endif

#ifdef AIRSPY_STREAM_WORK_TELEMETRY
  const uint32_t isr_elapsed = SCS_DWT_CYCCNT - isr_started;
  stream_contract->m4_dma_isr_cycles_total += isr_elapsed;
  if (isr_elapsed > stream_contract->m4_dma_isr_cycles_maximum)
  {
    stream_contract->m4_dma_isr_cycles_maximum = isr_elapsed;
  }
  stream_contract->m4_dma_isr_count++;
#endif
}

void m0core_isr(void)
{
  uint8_t adchs_conf;
  uint8_t adchs_start_stop_cmd;
  uint8_t samplerate_cmd;
  uint8_t packing_cmd;
  uint8_t packing_state;

  SLAVE_TXEV_QUIT();

  samplerate_cmd = get_samplerate(&adchs_conf);
  if(samplerate_cmd == SET_SAMPLERATE_CMD)
  {
    if((adchs_conf & AIRSPY_SAMPLERATE_CONF_ALT) == AIRSPY_SAMPLERATE_CONF_ALT)
    {
      adchs_conf = adchs_conf & (~AIRSPY_SAMPLERATE_CONF_ALT);
      sys_clock_samplerate(&airspy_conf->airspy_m0_m4_alt_conf[adchs_conf].airspy_m4_conf);
    }else
    {
      sys_clock_samplerate(&airspy_conf->airspy_m0_m4_conf[adchs_conf].airspy_m4_conf);
    }
    ack_samplerate();
  }

  packing_cmd = get_packing(&packing_state);
  if(packing_cmd == SET_PACKING_CMD)
  {
    set_packing_state(packing_state);
    ack_packing();
  }

  adchs_start_stop_cmd = get_start_stop_adchs();
  switch(adchs_start_stop_cmd)
  {
    case START_ADCHS_CMD:
      if(adchs_started == 0)
      {
        if(adchs_prepared == 0)
        {
          adchs_prepare();
        }
        adchs_start(DEFAULT_ADCHS_CHAN);
        adchs_started = 1;
        adchs_stopped = 0;
        adchs_prepared = 0;
      }
      ack_start_stop_adchs();
    break;

    case STOP_ADCHS_CMD:
      if(adchs_stopped == 0)
      {
        adchs_stop();
        adchs_stopped = 1;
        adchs_started = 0;
      }
      ack_start_stop_adchs();
    break;

    case PARK_PLL1_CMD:
      if(adchs_stopped != 0)
      {
        pll1_park();
      }
      ack_start_stop_adchs();
    break;

    case PREPARE_ADCHS_CMD:
      if(adchs_started == 0 && adchs_prepared == 0)
      {
        adchs_prepare();
      }
      ack_start_stop_adchs();
    break;

    default:
    /* Invalid command do nothing */
    break;
  }
}

void m0_startup(void)
{
  uint32_t *src, *dest;

  /* Halt M0 core (in case it was running) */
  ipc_halt_m0();

  /* Copy M0 code from M4 embedded addr to final addr M0 */
  dest = &cm0_exec_baseaddr;
  for(src = (uint32_t *)&m0_bin[0]; src < (uint32_t *)(&m0_bin[0]+m0_bin_size); )
  {
    *dest++ = *src++;
  }

  ipc_start_m0( (uint32_t)(&cm0_exec_baseaddr) );
}

void m0s_startup(void)
{
  uint32_t *src, *dest;

  /* Halt M0 core (in case it was running) */
  ipc_halt_m0s();

  /* Copy M0 code from M4 embedded addr to final addr M0 */
  dest = &cm0s_exec_baseaddr;
  for(src = (uint32_t *)&m0s_bin[0]; src < (uint32_t *)(&m0s_bin[0]+m0s_bin_size); )
  {
    *dest++ = *src++;
  }

  ipc_start_m0s( (uint32_t)(&cm0s_exec_baseaddr) );
}

void scs_dwt_cycle_counter_enabled(void)
{
  SCS_DEMCR |= SCS_DEMCR_TRCENA;
  SCS_DWT_CTRL  |= SCS_DWT_CTRL_CYCCNTENA;
}

int main(void)
{
  scs_dwt_cycle_counter_enabled();
  pin_setup();
  sys_clock_init();
  stream_contract_init();

  nvic_set_priority(NVIC_DMA_IRQ, 255);
  nvic_set_priority(NVIC_M0CORE_IRQ, 1);

  clr_usb_buffer_offset();

  nvic_enable_irq(NVIC_DMA_IRQ);
  nvic_enable_irq(NVIC_M0CORE_IRQ);

  adchs_stop();
  adchs_stopped = 1;
  adchs_started = 0;
  adchs_prepared = 0;

  use_packing = 0;
  *usb_bulk_buffer_length = 0x4000;

  ack_start_stop_adchs();
  ack_samplerate();
  ack_packing();

  /* Start M0 */
  m0_startup();

#undef ENABLE_M0S
#ifdef ENABLE_M0S
  /* Start M0s */
  m0s_startup();
#else
  // Halt M0s
  ipc_halt_m0s();
  // Disable M0 Sub
  CCU1_CLK_PERIPH_CORE_CFG &= ~(1);
#endif

  usb_bulk_buffer_offset_m4 = &usb_bulk_buffer_offset_uint32_m4;

  while(true)
  {
    signal_wfe();

    service_gpdma_probe();
    service_adc_recovery();
    service_adc_ring_resume();

    if(use_packing)
    {
      /* Thanks to Pierre HB9FUF for the initial packing proof-of-concept */
      uint32_t offset = get_usb_buffer_offset_m4();
      if(offset != last_offset_m4)
      {
        pack((uint32_t*)&usb_bulk_buffer[offset], (uint32_t*)&usb_bulk_buffer[offset], 0x1000);
        set_usb_buffer_offset( inc_mask_usb_buffer_offset(get_usb_buffer_offset(), 0x2000));
        signal_sev();
        last_offset_m4 = offset;
      }
    }
  }
}
