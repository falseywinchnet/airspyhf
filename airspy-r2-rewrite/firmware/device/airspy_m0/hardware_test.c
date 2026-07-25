#include "hardware_test.h"

#include "usb.h"
#include "usb_endpoint.h"
#include "usb_queue.h"
#include "airspy_commands.h"
#include "airspy_rx.h"

#include <libopencm3/lpc43xx/usb.h>

#include <signal_mcu.h>
#include <stream_contract.h>

#include <stdbool.h>
#include <stddef.h>

typedef struct {
  usb_atdtw_telemetry_t usb;
  uint32_t batches_scheduled;
  uint32_t schedule_failures;
  uint32_t forced_idle_observed;
  uint32_t forced_idle_timeouts;
  uint32_t ownership_generation;
  uint32_t ownership_submitted;
  uint32_t ownership_retired;
  uint32_t ownership_partial;
  uint32_t ownership_errors;
  uint32_t ownership_backpressure;
  uint32_t buffer_produced[AIRSPY_STREAM_BUFFER_COUNT];
  uint32_t buffer_submitted[AIRSPY_STREAM_BUFFER_COUNT];
  uint32_t buffer_retired[AIRSPY_STREAM_BUFFER_COUNT];
  uint32_t buffer_retired_bytes[AIRSPY_STREAM_BUFFER_COUNT];
} hardware_test_atdtw_telemetry_t;

static bool bulk_test_initialized;
static hardware_test_atdtw_telemetry_t atdtw_telemetry;

typedef struct {
  uint32_t command_generation;
  uint32_t destination_address;
  uint32_t status;
  uint32_t bytes;
  uint32_t cycles;
  uint32_t error_status;
  uint32_t expected_checksum;
  uint32_t actual_checksum;
  uint32_t mismatch_count;
  uint32_t first_mismatch_word;
} hardware_test_gpdma_result_t;

static hardware_test_gpdma_result_t gpdma_result;
static uint32_t ownership_generation;

extern uint32_t cm4_data_share;
static volatile airspy_stream_contract_t* const stream_contract =
  (airspy_stream_contract_t*)&cm4_data_share;

static uint32_t* const bulk_test_buffers[AIRSPY_HWTEST_BULK_BUFFERS] = {
  (uint32_t*)0x20004000u,
  (uint32_t*)0x20008000u,
  (uint32_t*)0x10018000u,
  (uint32_t*)0x10084000u,
};

static uint32_t* hardware_test_buffer(const uint16_t candidate)
{
  switch (candidate)
  {
    case AIRSPY_HWTEST_AHB_REFERENCE:
      return (uint32_t*)0x20004000u;
    case AIRSPY_HWTEST_LOCAL1:
      return (uint32_t*)0x10018000u;
    case AIRSPY_HWTEST_LOCAL2:
      return (uint32_t*)0x10084000u;
    case AIRSPY_HWTEST_M0SUB:
      return (uint32_t*)0x18000800u;
    default:
      return NULL;
  }
}

static uint32_t hardware_test_word(
  const uintptr_t address,
  const uint16_t seed,
  const uint32_t word_index)
{
  return (word_index * 0x9e3779b9u)
    ^ (uint32_t)address
    ^ ((uint32_t)seed << 16)
    ^ 0xa5c35a7eu;
}

static void hardware_test_fill(
  uint32_t* const buffer,
  const size_t bytes,
  const uint16_t seed)
{
  const size_t word_count = bytes / sizeof(uint32_t);
  for (size_t index = 0; index < word_count; ++index)
  {
    buffer[index] = hardware_test_word(
      (uintptr_t)buffer, seed, (uint32_t)index);
  }
}

static usb_request_status_t hardware_test_memory_read(
  usb_endpoint_t* const endpoint,
  const usb_transfer_stage_t stage)
{
  if (stage != USB_TRANSFER_STAGE_SETUP)
  {
    return USB_REQUEST_STATUS_OK;
  }
  if (endpoint->setup.length != AIRSPY_HWTEST_BUFFER_BYTES)
  {
    return USB_REQUEST_STATUS_STALL;
  }

  uint32_t* const buffer = hardware_test_buffer(endpoint->setup.value);
  if (buffer == NULL)
  {
    return USB_REQUEST_STATUS_STALL;
  }

  hardware_test_fill(
    buffer, AIRSPY_HWTEST_BUFFER_BYTES, endpoint->setup.index);

  __asm volatile ("dmb" ::: "memory");
  if (usb_transfer_schedule(
        endpoint->in, buffer, AIRSPY_HWTEST_BUFFER_BYTES) != 0)
  {
    return USB_REQUEST_STATUS_STALL;
  }
  usb_transfer_schedule_ack(endpoint->out);
  return USB_REQUEST_STATUS_OK;
}

static usb_request_status_t hardware_test_bulk_init(
  usb_endpoint_t* const endpoint,
  const usb_transfer_stage_t stage)
{
  if (stage != USB_TRANSFER_STAGE_SETUP)
  {
    return USB_REQUEST_STATUS_OK;
  }
  if (endpoint->setup.length != 0)
  {
    return USB_REQUEST_STATUS_STALL;
  }

  if (bulk_test_initialized)
  {
    usb_endpoint_resume(&usb_endpoint_bulk_in);
  }
  else
  {
    usb_endpoint_init(&usb_endpoint_bulk_in);
  }
  usb_atdtw_telemetry_reset();
  atdtw_telemetry = (hardware_test_atdtw_telemetry_t){0};
  stream_contract->usb_submitted = 0;
  stream_contract->usb_retired = 0;
  stream_contract->usb_backpressure = 0;
  stream_contract->usb_partial = 0;
  stream_contract->usb_errors = 0;
  stream_contract->usb_cancelled = 0;
  bulk_test_initialized = true;
  usb_transfer_schedule_ack(endpoint->in);
  return USB_REQUEST_STATUS_OK;
}

static void hardware_test_ownership_retired(
  void* const context,
  const uint32_t generation,
  const uint32_t actual_length,
  const usb_transfer_retirement_status_t status)
{
  volatile airspy_stream_buffer_record_t* const record = context;
  if (record->submitted_generation != generation)
  {
    record->flags |= AIRSPY_STREAM_BUFFER_FLAG_OVERWRITE_RISK;
    stream_contract->usb_errors++;
  }
  record->retired_bytes = actual_length;
  if (status == USB_TRANSFER_RETIREMENT_CANCELLED)
  {
    stream_contract->usb_cancelled++;
  }
  else if (status != USB_TRANSFER_RETIREMENT_COMPLETE)
  {
    stream_contract->usb_errors++;
  }
  else if (actual_length != AIRSPY_STREAM_BUFFER_BYTES)
  {
    stream_contract->usb_partial++;
  }
  airspy_stream_publish_barrier();
  record->retired_generation = generation;
  stream_contract->usb_retired++;
}

static usb_request_status_t hardware_test_atdtw_batch(
  usb_endpoint_t* const endpoint,
  const usb_transfer_stage_t stage)
{
  if (stage != USB_TRANSFER_STAGE_SETUP)
  {
    return USB_REQUEST_STATUS_OK;
  }
  if (!bulk_test_initialized
    || endpoint->setup.length != 0
    || endpoint->setup.value > AIRSPY_HWTEST_ATDTW_FORCE_REPRIME)
  {
    return USB_REQUEST_STATUS_STALL;
  }

  const bool force_reprime =
    endpoint->setup.value == AIRSPY_HWTEST_ATDTW_FORCE_REPRIME;
  for (size_t index = 0; index < AIRSPY_HWTEST_BULK_BUFFERS; ++index)
  {
    volatile airspy_stream_buffer_record_t* const record =
      &stream_contract->buffers[index];
    if (record->produced_generation != record->retired_generation
      || record->submitted_generation != record->retired_generation)
    {
      stream_contract->usb_backpressure++;
      return USB_REQUEST_STATUS_STALL;
    }
  }
  uint32_t generation = ownership_generation + 1u;
  if (generation == 0)
  {
    generation = 1;
  }
  ownership_generation = generation;
  stream_contract->mode = AIRSPY_STREAM_MODE_SYNTHETIC;

  for (size_t index = 0; index < AIRSPY_HWTEST_BULK_BUFFERS; ++index)
  {
    hardware_test_fill(
      bulk_test_buffers[index],
      AIRSPY_HWTEST_BUFFER_BYTES,
      (uint16_t)(endpoint->setup.index + index * 0x1111u));
  }
  __asm volatile ("dmb" ::: "memory");

  const uint32_t first_length = force_reprime
    ? AIRSPY_HWTEST_REPRIME_FIRST_BYTES
    : AIRSPY_HWTEST_BUFFER_BYTES;
  volatile airspy_stream_buffer_record_t* record =
    &stream_contract->buffers[0];
  record->produced_generation = generation;
  record->submitted_generation = generation;
  if (usb_transfer_schedule_tagged(
        &usb_endpoint_bulk_in, bulk_test_buffers[0], first_length,
        hardware_test_ownership_retired, (void*)record, generation) != 0)
  {
    record->produced_generation = record->retired_generation;
    record->submitted_generation = record->retired_generation;
    atdtw_telemetry.schedule_failures++;
    return USB_REQUEST_STATUS_STALL;
  }
  stream_contract->usb_submitted++;

  const uint32_t bulk_in_prime_mask = USB0_ENDPTPRIME_PETB(1 << 1);
  while (USB0_ENDPTPRIME & bulk_in_prime_mask);

  if (force_reprime)
  {
    const uint32_t bulk_in_active_mask = USB0_ENDPTSTAT_ETBR(1 << 1);
    uint32_t remaining = 1000000;
    while ((USB0_ENDPTSTAT & bulk_in_active_mask) && remaining != 0)
    {
      remaining--;
    }
    if (remaining == 0)
    {
      atdtw_telemetry.forced_idle_timeouts++;
    }
    else
    {
      atdtw_telemetry.forced_idle_observed++;
    }
  }

  for (size_t index = 1; index < AIRSPY_HWTEST_BULK_BUFFERS; ++index)
  {
    record = &stream_contract->buffers[index];
    record->produced_generation = generation;
    record->submitted_generation = generation;
    if (usb_transfer_schedule_tagged(
          &usb_endpoint_bulk_in,
          bulk_test_buffers[index],
          AIRSPY_HWTEST_BUFFER_BYTES,
          hardware_test_ownership_retired,
          (void*)record,
          generation) != 0)
    {
      record->produced_generation = record->retired_generation;
      record->submitted_generation = record->retired_generation;
      atdtw_telemetry.schedule_failures++;
      return USB_REQUEST_STATUS_STALL;
    }
    stream_contract->usb_submitted++;
  }

  stream_contract->capture_generation = generation;
  stream_contract->capture_completed += AIRSPY_HWTEST_BULK_BUFFERS;
  atdtw_telemetry.batches_scheduled++;
  usb_transfer_schedule_ack(endpoint->in);
  return USB_REQUEST_STATUS_OK;
}

static usb_request_status_t hardware_test_atdtw_telemetry(
  usb_endpoint_t* const endpoint,
  const usb_transfer_stage_t stage)
{
  if (stage != USB_TRANSFER_STAGE_SETUP)
  {
    return USB_REQUEST_STATUS_OK;
  }
  if (endpoint->setup.length != sizeof(atdtw_telemetry))
  {
    return USB_REQUEST_STATUS_STALL;
  }

  usb_atdtw_telemetry_snapshot(&atdtw_telemetry.usb);
  atdtw_telemetry.ownership_generation =
    stream_contract->capture_generation;
  atdtw_telemetry.ownership_submitted = stream_contract->usb_submitted;
  atdtw_telemetry.ownership_retired = stream_contract->usb_retired;
  atdtw_telemetry.ownership_partial = stream_contract->usb_partial;
  atdtw_telemetry.ownership_errors = stream_contract->usb_errors;
  atdtw_telemetry.ownership_backpressure =
    stream_contract->usb_backpressure;
  for (size_t index = 0; index < AIRSPY_STREAM_BUFFER_COUNT; ++index)
  {
    atdtw_telemetry.buffer_produced[index] =
      stream_contract->buffers[index].produced_generation;
    atdtw_telemetry.buffer_submitted[index] =
      stream_contract->buffers[index].submitted_generation;
    atdtw_telemetry.buffer_retired[index] =
      stream_contract->buffers[index].retired_generation;
    atdtw_telemetry.buffer_retired_bytes[index] =
      stream_contract->buffers[index].retired_bytes;
  }
  if (usb_transfer_schedule(
        endpoint->in, &atdtw_telemetry, sizeof(atdtw_telemetry)) != 0)
  {
    return USB_REQUEST_STATUS_STALL;
  }
  usb_transfer_schedule_ack(endpoint->out);
  return USB_REQUEST_STATUS_OK;
}

static usb_request_status_t hardware_test_gpdma_probe(
  usb_endpoint_t* const endpoint,
  const usb_transfer_stage_t stage)
{
  if (stage != USB_TRANSFER_STAGE_SETUP)
  {
    return USB_REQUEST_STATUS_OK;
  }
  if (endpoint->setup.length != sizeof(gpdma_result)
    || endpoint->setup.value < AIRSPY_HWTEST_LOCAL1
    || endpoint->setup.value > AIRSPY_HWTEST_LOCAL2
    || get_receiver_mode() != RECEIVER_MODE_OFF
    || stream_contract->magic != AIRSPY_STREAM_CONTRACT_MAGIC)
  {
    return USB_REQUEST_STATUS_STALL;
  }

  uint32_t* const source = hardware_test_buffer(AIRSPY_HWTEST_AHB_REFERENCE);
  uint32_t* const destination = hardware_test_buffer(endpoint->setup.value);
  hardware_test_fill(
    source, AIRSPY_HWTEST_BUFFER_BYTES, endpoint->setup.index);
  for (size_t index = 0;
    index < AIRSPY_HWTEST_BUFFER_BYTES / sizeof(uint32_t); ++index)
  {
    destination[index] = 0;
  }

  uint32_t command = stream_contract->gpdma_command_generation + 1u;
  if (command == 0)
  {
    command = 1;
  }
  stream_contract->gpdma_destination_index = endpoint->setup.value + 1u;
  stream_contract->gpdma_status = AIRSPY_STREAM_GPDMA_PENDING;
  airspy_stream_publish_barrier();
  stream_contract->gpdma_command_generation = command;
  signal_sev();

  uint32_t timeout = 100u * 1000u * 1000u;
  while (stream_contract->gpdma_completed_generation != command
    && timeout != 0)
  {
    timeout--;
  }

  gpdma_result.command_generation = command;
  gpdma_result.destination_address = (uint32_t)destination;
  gpdma_result.status = timeout != 0
    ? stream_contract->gpdma_status
    : AIRSPY_STREAM_GPDMA_TIMEOUT;
  gpdma_result.bytes = stream_contract->gpdma_bytes;
  gpdma_result.cycles = stream_contract->gpdma_cycles;
  gpdma_result.error_status = stream_contract->gpdma_error_status;
  gpdma_result.expected_checksum =
    stream_contract->gpdma_expected_checksum;
  gpdma_result.actual_checksum = stream_contract->gpdma_actual_checksum;
  gpdma_result.mismatch_count = 0;
  gpdma_result.first_mismatch_word = 0xffffffffu;

  for (uint32_t index = 0;
    index < AIRSPY_HWTEST_BUFFER_BYTES / sizeof(uint32_t); ++index)
  {
    if (source[index] != destination[index])
    {
      if (gpdma_result.mismatch_count == 0)
      {
        gpdma_result.first_mismatch_word = index;
      }
      gpdma_result.mismatch_count++;
    }
  }

  if (usb_transfer_schedule(
        endpoint->in, &gpdma_result, sizeof(gpdma_result)) != 0)
  {
    return USB_REQUEST_STATUS_STALL;
  }
  usb_transfer_schedule_ack(endpoint->out);
  return USB_REQUEST_STATUS_OK;
}

usb_request_status_t usb_vendor_request_hardware_test(
  usb_endpoint_t* const endpoint,
  const usb_transfer_stage_t stage)
{
  switch (endpoint->setup.request)
  {
    case AIRSPY_HWTEST_MEMORY_READ:
      return hardware_test_memory_read(endpoint, stage);
    case AIRSPY_HWTEST_BULK_INIT:
      return hardware_test_bulk_init(endpoint, stage);
    case AIRSPY_HWTEST_ATDTW_BATCH:
      return hardware_test_atdtw_batch(endpoint, stage);
    case AIRSPY_HWTEST_ATDTW_TELEMETRY:
      return hardware_test_atdtw_telemetry(endpoint, stage);
    case AIRSPY_HWTEST_GPDMA_PROBE:
      return hardware_test_gpdma_probe(endpoint, stage);
    default:
      return USB_REQUEST_STATUS_STALL;
  }
}
