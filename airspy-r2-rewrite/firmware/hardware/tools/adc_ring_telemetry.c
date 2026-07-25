#include <libusb.h>

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

enum {
  AIRSPY_VID = 0x1d50,
  AIRSPY_PID = 0x60a1,
  STREAM_TELEMETRY_REQUEST = 0x87,
  STREAM_MAGIC = 0x53424f34u,
  STREAM_VERSION = 9,
  STREAM_MODE_ADC_FOUR_BUFFER = 2,
  STREAM_MODE_RECOVERING = 3,
  BUFFER_COUNT = 10,
  RETIRE_QUEUE_COUNT = 16,
  GRANT_QUEUE_COUNT = 16,
  USB_TIMEOUT_MS = 5000
};

typedef struct {
  uint32_t address;
  uint32_t produced_generation;
  uint32_t dma_start_cycles;
  uint32_t dma_complete_cycles;
  uint32_t flags;
  uint32_t granted_generation;
  uint32_t submitted_generation;
  uint32_t retired_generation;
  uint32_t retired_bytes;
} buffer_record_t;

typedef struct {
  uint32_t legacy[3];
  uint32_t magic;
  uint32_t version;
  uint32_t mode;
  uint32_t buffer_count;
  uint32_t buffer_bytes;
  uint32_t capture_generation;
  uint32_t capture_completed;
  uint32_t capture_halted;
  uint32_t overwrite_prevented;
  uint32_t dma_error_count;
  uint32_t ownership_overwrite_count;
  uint32_t adc_fifo_overflow_count;
  uint32_t adc_descriptor_error_count;
  uint32_t adc_overrange_count;
  uint32_t adc_underrange_count;
  uint32_t minimum_completion_cycles;
  uint32_t maximum_completion_cycles;
  uint32_t usb_submitted;
  uint32_t usb_retired;
  uint32_t usb_backpressure;
  uint32_t usb_partial;
  uint32_t usb_errors;
  uint32_t usb_cancelled;
  uint32_t usb_suspended;
  uint32_t usb_suspend_count;
  uint32_t usb_resume_count;
  uint32_t usb_suspend_discontinuities;
  uint32_t usb_queue_recovery_count;
  uint32_t usb_partial_discontinuity_count;
  uint32_t usb_controller_error_irq_count;
  uint32_t usb_bus_reset_count;
  uint32_t usb_port_change_count;
  uint32_t retire_queue_write_sequence;
  uint32_t retire_queue_read_sequence;
  uint32_t retire_queue_overflows;
  uint32_t retire_queue_entries[RETIRE_QUEUE_COUNT];
  uint32_t grant_queue_write_sequence;
  uint32_t grant_queue_read_sequence;
  uint32_t grant_queue_overflows;
  uint32_t grant_queue_entries[GRANT_QUEUE_COUNT];
  uint32_t recovery_request_generation;
  uint32_t recovery_acknowledged_generation;
  uint32_t recovery_completed_generation;
  uint32_t dma_recovery_count;
  uint32_t dma_recovery_failure_count;
  uint32_t dma_recovery_dropped_buffer_estimate;
  uint32_t last_dma_error_status;
  uint32_t backpressure_discontinuity_count;
  uint32_t stream_poisoned;
  uint32_t stream_poison_count;
  uint32_t poison_transport_terminated;
  uint32_t adc_fifo_level_high_water;
  uint32_t adc_fifo_full_observations;
  uint32_t usb_system_error_count;
  uint32_t gpdma[9];
  uint32_t clock_stream_pll1_ctrl;
  uint32_t clock_idle_pll1_ctrl;
  uint32_t clock_base_m4;
  uint32_t clock_base_periph;
  uint32_t clock_base_apb1;
  uint32_t clock_base_apb3;
  uint32_t clock_high_transitions;
  uint32_t clock_low_transitions;
  uint32_t m4_dma_isr_cycles_total;
  uint32_t m4_dma_isr_cycles_maximum;
  uint32_t m4_dma_isr_count;
  uint32_t m4_resume_cycles_total;
  uint32_t m4_resume_cycles_maximum;
  uint32_t m4_resume_count;
  uint32_t m0_submit_cycles_total;
  uint32_t m0_submit_cycles_maximum;
  uint32_t m0_submit_count;
  uint32_t m0_retire_cycles_total;
  uint32_t m0_retire_cycles_maximum;
  uint32_t m0_retire_count;
  uint32_t steering_decisions;
  uint32_t steering_overwrites;
  uint32_t steering_overwrite_runs;
  uint32_t steering_overwrite_run_current;
  uint32_t steering_overwrite_run_maximum;
  uint32_t steering_alternation_violations;
  uint32_t steering_no_candidate_faults;
  uint32_t steering_group_skips;
  uint32_t steering_minimum_available;
  uint32_t steering_minimum_groups;
  uint32_t steering_current_available;
  uint32_t steering_current_groups;
  uint32_t steering_floor_boundaries;
  uint32_t steering_floor_fast_path_boundaries;
  uint32_t steering_full_scan_boundaries;
  uint32_t steering_isr_cycles_maximum;
  uint32_t maximum_capture_to_grant_age;
  uint32_t stale_generation_completions;
  uint32_t steering_available_histogram[BUFFER_COUNT + 1];
  buffer_record_t buffers[BUFFER_COUNT];
} stream_contract_t;

_Static_assert(sizeof(stream_contract_t) == 940,
  "host and firmware stream contracts must agree");

static const char* stream_mode_name(const uint32_t mode)
{
  switch (mode)
  {
    case 0: return "legacy";
    case 1: return "synthetic";
    case STREAM_MODE_ADC_FOUR_BUFFER: return "ADC ring";
    case STREAM_MODE_RECOVERING: return "recovering";
    case 4: return "poisoned";
    default: return "unknown";
  }
}

static int read_contract(
  libusb_device_handle* const handle,
  stream_contract_t* const contract)
{
  const int result = libusb_control_transfer(
    handle,
    LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR
      | LIBUSB_RECIPIENT_DEVICE,
    STREAM_TELEMETRY_REQUEST, 0, 0,
    (unsigned char*)contract, sizeof(*contract), USB_TIMEOUT_MS);
  if (result != sizeof(*contract))
  {
    fprintf(stderr, "Telemetry read failed: %s\n",
      result < 0 ? libusb_error_name(result) : "short transfer");
    return -1;
  }
  if (contract->magic != STREAM_MAGIC || contract->version != STREAM_VERSION)
  {
    fprintf(stderr,
      "Telemetry contract mismatch: magic=%08" PRIx32
        ", version=%" PRIu32 " (expected %08x/%u)\n",
      contract->magic, contract->version, STREAM_MAGIC, STREAM_VERSION);
    return -1;
  }
  return 0;
}

static void print_fault_deltas(
  const stream_contract_t* const current,
  const stream_contract_t* const previous)
{
#define DELTA(field) (current->field - previous->field)
  printf(
    "sample: mode=%s, capture +%" PRIu32 ", USB retire +%" PRIu32
      ", outstanding=%" PRIu32 "\n",
    stream_mode_name(current->mode), DELTA(capture_completed),
    DELTA(usb_retired), current->usb_submitted - current->usb_retired);

  if (DELTA(adc_fifo_overflow_count) != 0)
  {
    printf("  ADC discontinuity: FIFO overflow +%" PRIu32 "\n",
      DELTA(adc_fifo_overflow_count));
  }
  if (DELTA(adc_descriptor_error_count) != 0)
  {
    printf("  ADC descriptor fault +%" PRIu32 "\n",
      DELTA(adc_descriptor_error_count));
  }
  if (DELTA(adc_overrange_count) != 0 || DELTA(adc_underrange_count) != 0)
  {
    printf("  ADC range flags: over +%" PRIu32 ", under +%" PRIu32
      " (signal/ADC condition, not necessarily transport loss)\n",
      DELTA(adc_overrange_count), DELTA(adc_underrange_count));
  }
  if (DELTA(dma_error_count) != 0 || DELTA(dma_recovery_count) != 0
    || DELTA(dma_recovery_failure_count) != 0)
  {
    printf(
      "  DMA: errors +%" PRIu32 ", recovered +%" PRIu32
        ", recovery failures +%" PRIu32 ", estimated dropped buffers +%" PRIu32
        ", last status=%08" PRIx32 "\n",
      DELTA(dma_error_count), DELTA(dma_recovery_count),
      DELTA(dma_recovery_failure_count),
      DELTA(dma_recovery_dropped_buffer_estimate),
      current->last_dma_error_status);
  }
  if (DELTA(usb_errors) != 0 || DELTA(usb_queue_recovery_count) != 0
    || DELTA(usb_partial_discontinuity_count) != 0
    || DELTA(usb_controller_error_irq_count) != 0)
  {
    printf(
      "  USB: dTD errors +%" PRIu32 ", queue recoveries +%" PRIu32
        ", partial discontinuities +%" PRIu32
        ", controller error IRQs +%" PRIu32 "\n",
      DELTA(usb_errors), DELTA(usb_queue_recovery_count),
      DELTA(usb_partial_discontinuity_count),
      DELTA(usb_controller_error_irq_count));
  }
  if (DELTA(usb_backpressure) != 0)
  {
    printf(
      "  USB backpressure +%" PRIu32
        ", resulting ADC discontinuities +%" PRIu32 "\n",
      DELTA(usb_backpressure), DELTA(backpressure_discontinuity_count));
  }
  if (DELTA(usb_suspend_discontinuities) != 0)
  {
    printf("  USB suspend/resume discontinuities +%" PRIu32 "\n",
      DELTA(usb_suspend_discontinuities));
  }
  if (DELTA(usb_bus_reset_count) != 0 || DELTA(usb_port_change_count) != 0)
  {
    printf("  USB link state: bus resets +%" PRIu32
      ", port changes +%" PRIu32 "\n",
      DELTA(usb_bus_reset_count), DELTA(usb_port_change_count));
  }
  if (DELTA(ownership_overwrite_count) != 0
    || DELTA(overwrite_prevented) != 0)
  {
    printf(
      "  Ownership guard: overwrite attempts +%" PRIu32
        ", captures halted safely +%" PRIu32 "\n",
      DELTA(ownership_overwrite_count), DELTA(overwrite_prevented));
  }
  if (DELTA(steering_overwrites) != 0)
  {
    printf(
      "  Steering: deliberately discarded +%" PRIu32
        " banks in +%" PRIu32 " consecutive runs"
        " (longest run=%" PRIu32 ")\n",
      DELTA(steering_overwrites), DELTA(steering_overwrite_runs),
      current->steering_overwrite_run_maximum);
  }
  if (DELTA(steering_alternation_violations) != 0
    || DELTA(steering_no_candidate_faults) != 0
    || DELTA(stale_generation_completions) != 0)
  {
    printf(
      "  Steering faults: alternation +%" PRIu32
        ", no-candidate +%" PRIu32 ", stale completion +%" PRIu32 "\n",
      DELTA(steering_alternation_violations),
      DELTA(steering_no_candidate_faults),
      DELTA(stale_generation_completions));
  }
#undef DELTA
  fflush(stdout);
}

static libusb_device_handle* open_airspy(
  libusb_context* const context,
  const char* const expected_serial)
{
  libusb_device** devices = NULL;
  const ssize_t count = libusb_get_device_list(context, &devices);
  if (count < 0)
  {
    return NULL;
  }

  libusb_device_handle* selected = NULL;
  for (ssize_t index = 0; index < count && selected == NULL; ++index)
  {
    struct libusb_device_descriptor descriptor;
    if (libusb_get_device_descriptor(devices[index], &descriptor) != 0
      || descriptor.idVendor != AIRSPY_VID
      || descriptor.idProduct != AIRSPY_PID)
    {
      continue;
    }
    libusb_device_handle* handle = NULL;
    if (libusb_open(devices[index], &handle) != 0)
    {
      continue;
    }
    char serial[256] = {0};
    if (descriptor.iSerialNumber != 0)
    {
      libusb_get_string_descriptor_ascii(
        handle, descriptor.iSerialNumber, (unsigned char*)serial,
        sizeof(serial) - 1);
    }
    if (expected_serial != NULL && strstr(serial, expected_serial) == NULL)
    {
      libusb_close(handle);
      continue;
    }
    printf("Opened Airspy: %s, USB speed %d\n", serial,
      libusb_get_device_speed(devices[index]));
    selected = handle;
  }
  libusb_free_device_list(devices, 1);
  return selected;
}

int main(int argc, char** argv)
{
  int reset = 0;
  int watch = 0;
  uint32_t interval_ms = 1000;
  const char* serial = NULL;
  for (int index = 1; index < argc; ++index)
  {
    if (strcmp(argv[index], "--reset") == 0)
    {
      reset = 1;
    }
    else if (strcmp(argv[index], "--watch") == 0)
    {
      watch = 1;
    }
    else if (strcmp(argv[index], "--serial") == 0 && index + 1 < argc)
    {
      serial = argv[++index];
    }
    else if (strcmp(argv[index], "--interval-ms") == 0
      && index + 1 < argc)
    {
      char* end = NULL;
      const unsigned long parsed = strtoul(argv[++index], &end, 10);
      if (end == argv[index] || *end != '\0' || parsed == 0
        || parsed > 60000)
      {
        fprintf(stderr, "Invalid interval: %s\n", argv[index]);
        return EXIT_FAILURE;
      }
      interval_ms = (uint32_t)parsed;
    }
    else
    {
      fprintf(stderr,
        "Usage: %s [--watch] [--interval-ms 1..60000]"
          " [--serial TEXT]\n"
        "       %s --reset [--serial TEXT]\n",
        argv[0], argv[0]);
      return EXIT_FAILURE;
    }
  }
  if (reset && watch)
  {
    fputs("--reset and --watch cannot be combined\n", stderr);
    return EXIT_FAILURE;
  }

  libusb_context* context = NULL;
  if (libusb_init(&context) != 0)
  {
    return EXIT_FAILURE;
  }
  libusb_device_handle* const handle = open_airspy(context, serial);
  if (handle == NULL)
  {
    libusb_exit(context);
    return EXIT_FAILURE;
  }
  if (reset)
  {
    const int reset_result = libusb_control_transfer(
      handle, LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR
        | LIBUSB_RECIPIENT_DEVICE,
      0, 0, 0, NULL, 0, USB_TIMEOUT_MS);
    libusb_close(handle);
    libusb_exit(context);
    printf("Reset request: %s\n",
      reset_result < 0 ? libusb_error_name(reset_result) : "sent");
    return reset_result < 0 && reset_result != LIBUSB_ERROR_NO_DEVICE
      ? EXIT_FAILURE : EXIT_SUCCESS;
  }

  stream_contract_t contract = {0};
  if (read_contract(handle, &contract) != 0)
  {
    libusb_close(handle);
    libusb_exit(context);
    return EXIT_FAILURE;
  }

  printf(
    "contract: version=%" PRIu32 ", mode=%s\n"
    "capture: generation=%" PRIu32 ", completed=%" PRIu32
      ", halted=%" PRIu32 ", overwrite-prevented=%" PRIu32
      ", DMA-errors=%" PRIu32 ", ownership-overwrites=%" PRIu32
      ", ADC-FIFO-overflows=%" PRIu32
      ", descriptor-errors=%" PRIu32
      ", over/under-range=%" PRIu32 "/%" PRIu32 "\n"
    "timing: completion cycles min/max=%" PRIu32 "/%" PRIu32 "\n"
    "USB: submitted=%" PRIu32 ", retired=%" PRIu32
      ", cancelled=%" PRIu32 ", partial=%" PRIu32
      ", errors=%" PRIu32 ", backpressure=%" PRIu32
      ", queue-recoveries=%" PRIu32
      ", partial-discontinuities=%" PRIu32
      ", controller-error-IRQs=%" PRIu32
      ", bus-resets=%" PRIu32 ", port-changes=%" PRIu32 "\n"
    "retirement notifications: write/read=%" PRIu32 "/%" PRIu32
      ", overflows=%" PRIu32 "\n"
    "grant notifications: write/read=%" PRIu32 "/%" PRIu32
      ", overflows=%" PRIu32 "\n"
    "phase safety: poisoned=%" PRIu32 ", poison-count=%" PRIu32
      ", transport-terminated=%" PRIu32
      ", FIFO-level-high-water=%" PRIu32 "/16"
      ", full-observations=%" PRIu32
      ", USB-system-errors=%" PRIu32 "\n"
    "suspend: active=%" PRIu32 ", suspend/resume=%" PRIu32 "/%" PRIu32
      ", discontinuities=%" PRIu32 "\n"
    "recovery: request/ack/complete=%" PRIu32 "/%" PRIu32 "/%" PRIu32
      ", recovered=%" PRIu32 ", failures=%" PRIu32
      ", estimated-dropped=%" PRIu32 ", last-DMA-status=%08" PRIx32
      ", backpressure-discontinuities=%" PRIu32 "\n"
    "clock: stream PLL1=%08" PRIx32 ", idle PLL1=%08" PRIx32
      ", transitions high/low=%" PRIu32 "/%" PRIu32 "\n"
    "clock bases: M4=%08" PRIx32 ", peripheral=%08" PRIx32
      ", APB1=%08" PRIx32 ", APB3=%08" PRIx32 "\n"
    "work cycles average/max: M4 ISR=%" PRIu32 "/%" PRIu32
      ", resume=%" PRIu32 "/%" PRIu32
      ", M0 submit=%" PRIu32 "/%" PRIu32
      ", M0 retire=%" PRIu32 "/%" PRIu32 "\n"
    "steering: decisions=%" PRIu32 ", deliberate-overwrites=%" PRIu32
      ", runs=%" PRIu32 ", longest-run=%" PRIu32
      ", alternation/no-candidate/stale=%" PRIu32 "/%" PRIu32 "/%" PRIu32
      ", current available/groups=%" PRIu32 "/%" PRIu32
      ", minimum available/groups=%" PRIu32 "/%" PRIu32
      ", floor/fast/full=%" PRIu32 "/%" PRIu32 "/%" PRIu32
      ", worst cycles=%" PRIu32
      ", maximum grant age=%" PRIu32 " banks\n",
    contract.version, stream_mode_name(contract.mode),
    contract.capture_generation, contract.capture_completed,
    contract.capture_halted, contract.overwrite_prevented,
    contract.dma_error_count, contract.ownership_overwrite_count,
    contract.adc_fifo_overflow_count, contract.adc_descriptor_error_count,
    contract.adc_overrange_count, contract.adc_underrange_count,
    contract.minimum_completion_cycles,
    contract.maximum_completion_cycles, contract.usb_submitted,
    contract.usb_retired, contract.usb_cancelled, contract.usb_partial,
    contract.usb_errors, contract.usb_backpressure,
    contract.usb_queue_recovery_count,
    contract.usb_partial_discontinuity_count,
    contract.usb_controller_error_irq_count,
    contract.usb_bus_reset_count, contract.usb_port_change_count,
    contract.retire_queue_write_sequence,
    contract.retire_queue_read_sequence,
    contract.retire_queue_overflows,
    contract.grant_queue_write_sequence,
    contract.grant_queue_read_sequence,
    contract.grant_queue_overflows,
    contract.stream_poisoned,
    contract.stream_poison_count,
    contract.poison_transport_terminated,
    contract.adc_fifo_level_high_water,
    contract.adc_fifo_full_observations,
    contract.usb_system_error_count,
    contract.usb_suspended, contract.usb_suspend_count,
    contract.usb_resume_count, contract.usb_suspend_discontinuities,
    contract.recovery_request_generation,
    contract.recovery_acknowledged_generation,
    contract.recovery_completed_generation, contract.dma_recovery_count,
    contract.dma_recovery_failure_count,
    contract.dma_recovery_dropped_buffer_estimate,
    contract.last_dma_error_status,
    contract.backpressure_discontinuity_count,
    contract.clock_stream_pll1_ctrl, contract.clock_idle_pll1_ctrl,
    contract.clock_high_transitions, contract.clock_low_transitions,
    contract.clock_base_m4, contract.clock_base_periph,
    contract.clock_base_apb1, contract.clock_base_apb3,
    contract.m4_dma_isr_count == 0 ? 0
      : contract.m4_dma_isr_cycles_total / contract.m4_dma_isr_count,
    contract.m4_dma_isr_cycles_maximum,
    contract.m4_resume_count == 0 ? 0
      : contract.m4_resume_cycles_total / contract.m4_resume_count,
    contract.m4_resume_cycles_maximum,
    contract.m0_submit_count == 0 ? 0
      : contract.m0_submit_cycles_total / contract.m0_submit_count,
    contract.m0_submit_cycles_maximum,
    contract.m0_retire_count == 0 ? 0
      : contract.m0_retire_cycles_total / contract.m0_retire_count,
    contract.m0_retire_cycles_maximum,
    contract.steering_decisions, contract.steering_overwrites,
    contract.steering_overwrite_runs,
    contract.steering_overwrite_run_maximum,
    contract.steering_alternation_violations,
    contract.steering_no_candidate_faults,
    contract.stale_generation_completions,
    contract.steering_current_available,
    contract.steering_current_groups,
    contract.steering_minimum_available,
    contract.steering_minimum_groups,
    contract.steering_floor_boundaries,
    contract.steering_floor_fast_path_boundaries,
    contract.steering_full_scan_boundaries,
    contract.steering_isr_cycles_maximum,
    contract.maximum_capture_to_grant_age);

  if (watch)
  {
    puts("Watching counters; Ctrl-C stops. A line without an event means the"
      " interval completed cleanly.");
    stream_contract_t previous = contract;
    while (1)
    {
      const struct timespec delay = {
        .tv_sec = interval_ms / 1000,
        .tv_nsec = (long)(interval_ms % 1000) * 1000000L
      };
      nanosleep(&delay, NULL);
      if (read_contract(handle, &contract) != 0)
      {
        libusb_close(handle);
        libusb_exit(context);
        return EXIT_FAILURE;
      }
      print_fault_deltas(&contract, &previous);
      previous = contract;
    }
  }

  int failed = contract.magic != STREAM_MAGIC
    || contract.version != STREAM_VERSION
    || contract.mode != STREAM_MODE_ADC_FOUR_BUFFER
    || contract.capture_completed == 0
    || contract.capture_halted != 0
    || contract.dma_error_count != 0
    || contract.usb_submitted != contract.usb_retired
    || contract.usb_partial != 0
    || contract.usb_errors != 0
    || contract.usb_backpressure != 0;
  for (size_t index = 0; index < BUFFER_COUNT; ++index)
  {
    const buffer_record_t* const record = &contract.buffers[index];
    printf(
      "buffer %zu @ %08" PRIx32 ": produced=%" PRIu32
        ", granted=%" PRIu32 ", submitted=%" PRIu32 ", retired=%" PRIu32
        ", bytes=%" PRIu32 ", flags=%08" PRIx32 "\n",
      index, record->address, record->produced_generation,
      record->granted_generation,
      record->submitted_generation, record->retired_generation,
      record->retired_bytes, record->flags);
    if (record->produced_generation != record->granted_generation
      || record->granted_generation != record->submitted_generation
      || record->submitted_generation != record->retired_generation
      || record->flags != 0)
    {
      failed = 1;
    }
  }

  if (contract.ownership_overwrite_count != 0)
  {
    puts("FAIL: an owned ring buffer was overwritten.");
  }
  else if (contract.adc_fifo_overflow_count != 0)
  {
    puts("FAIL: ownership remained ordered, but bus stalls lost ADC samples.");
  }
  else
  {
    puts("PASS: live ADC ring ownership is balanced without sample loss.");
  }
  libusb_close(handle);
  libusb_exit(context);
  return failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
