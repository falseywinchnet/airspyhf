#include <libusb.h>

#include <errno.h>
#include <inttypes.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

enum {
  AIRSPY_VID = 0x1d50,
  AIRSPY_PID = 0x60a1,
  BULK_INIT_REQUEST = 0x81,
  ATDTW_BATCH_REQUEST = 0x82,
  ATDTW_TELEMETRY_REQUEST = 0x83,
  BULK_IN_ENDPOINT = 0x81,
  BUFFER_BYTES = 16 * 1024,
  BUFFER_COUNT = 4,
  REPRIME_FIRST_BYTES = 512,
  ACTIVE_MODE = 0,
  FORCE_REPRIME_MODE = 1,
  DEFAULT_ACTIVE_BATCHES = 1000,
  DEFAULT_REPRIME_BATCHES = 100,
  USB_TIMEOUT_MS = 5000
};

static const uintptr_t buffer_addresses[BUFFER_COUNT] = {
  0x20004000u,
  0x20008000u,
  0x10018000u,
  0x10084000u,
};

typedef struct {
  uint32_t append_calls;
  uint32_t prime_in_progress_exits;
  uint32_t tripwire_samples;
  uint32_t tripwire_retries;
  uint32_t endpoint_active_exits;
  uint32_t endpoint_reprime_exits;
  uint32_t completion_callbacks;
  uint32_t descriptors_retired;
  uint32_t maximum_retired_per_callback;
  uint32_t descriptor_errors;
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
  uint32_t buffer_produced[BUFFER_COUNT];
  uint32_t buffer_submitted[BUFFER_COUNT];
  uint32_t buffer_retired[BUFFER_COUNT];
  uint32_t buffer_retired_bytes[BUFFER_COUNT];
} atdtw_telemetry_t;

_Static_assert(sizeof(atdtw_telemetry_t) == 36 * sizeof(uint32_t),
  "firmware and host telemetry layouts must agree");

typedef struct {
  libusb_device_handle* handle;
  uint8_t* buffer;
  int length;
  int transferred;
  int result;
} bulk_thread_args_t;

static uint32_t load_le32(const uint8_t* const bytes)
{
  return ((uint32_t)bytes[0])
    | ((uint32_t)bytes[1] << 8)
    | ((uint32_t)bytes[2] << 16)
    | ((uint32_t)bytes[3] << 24);
}

static uint32_t expected_word(
  const uintptr_t address,
  const uint16_t seed,
  const uint32_t word_index)
{
  return (word_index * UINT32_C(0x9e3779b9))
    ^ (uint32_t)address
    ^ ((uint32_t)seed << 16)
    ^ UINT32_C(0xa5c35a7e);
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
      const int length = libusb_get_string_descriptor_ascii(
        handle, descriptor.iSerialNumber, (unsigned char*)serial,
        sizeof(serial) - 1);
      if (length < 0)
      {
        serial[0] = '\0';
      }
    }
    if (expected_serial != NULL && strstr(serial, expected_serial) == NULL)
    {
      libusb_close(handle);
      continue;
    }

    printf("Opened Airspy: %s, USB speed %d\n",
      serial[0] != '\0' ? serial : "(serial unavailable)",
      libusb_get_device_speed(devices[index]));
    selected = handle;
  }

  libusb_free_device_list(devices, 1);
  return selected;
}

static int vendor_out(
  libusb_device_handle* const handle,
  const uint8_t request,
  const uint16_t value,
  const uint16_t index)
{
  const int result = libusb_control_transfer(
    handle,
    LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR
      | LIBUSB_RECIPIENT_DEVICE,
    request, value, index, NULL, 0, USB_TIMEOUT_MS);
  if (result != 0)
  {
    fprintf(stderr, "Vendor request 0x%02x failed: %s\n",
      request, libusb_error_name(result));
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

static int read_telemetry(
  libusb_device_handle* const handle,
  atdtw_telemetry_t* const telemetry)
{
  const int result = libusb_control_transfer(
    handle,
    LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR
      | LIBUSB_RECIPIENT_DEVICE,
    ATDTW_TELEMETRY_REQUEST, 0, 0,
    (unsigned char*)telemetry, sizeof(*telemetry), USB_TIMEOUT_MS);
  if (result != sizeof(*telemetry))
  {
    fprintf(stderr, "Telemetry read failed: %s\n",
      result < 0 ? libusb_error_name(result) : "short transfer");
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

static void* bulk_thread(void* opaque)
{
  bulk_thread_args_t* const args = opaque;
  args->result = libusb_bulk_transfer(
    args->handle, BULK_IN_ENDPOINT, args->buffer, args->length,
    &args->transferred, USB_TIMEOUT_MS);
  return NULL;
}

static int verify_batch(
  const uint8_t* const received,
  const uint16_t batch_seed,
  const bool force_reprime)
{
  size_t received_offset = 0;
  for (size_t buffer_index = 0;
    buffer_index < BUFFER_COUNT; ++buffer_index)
  {
    const size_t segment_bytes = force_reprime && buffer_index == 0
      ? REPRIME_FIRST_BYTES
      : BUFFER_BYTES;
    const uint16_t seed = (uint16_t)(
      batch_seed + buffer_index * UINT16_C(0x1111));

    for (uint32_t word_index = 0;
      word_index < segment_bytes / sizeof(uint32_t); ++word_index)
    {
      const uint32_t actual = load_le32(
        received + received_offset + word_index * 4);
      const uint32_t expected = expected_word(
        buffer_addresses[buffer_index], seed, word_index);
      if (actual != expected)
      {
        fprintf(stderr,
          "Data mismatch in buffer %zu, word %" PRIu32
          ": expected %08" PRIx32 ", got %08" PRIx32 "\n",
          buffer_index, word_index, expected, actual);
        return EXIT_FAILURE;
      }
    }
    received_offset += segment_bytes;
  }
  return EXIT_SUCCESS;
}

static int receive_active_batch(
  libusb_device_handle* const handle,
  uint8_t* const receive_buffer,
  const uint16_t seed)
{
  if (vendor_out(handle, ATDTW_BATCH_REQUEST, ACTIVE_MODE, seed)
    != EXIT_SUCCESS)
  {
    return EXIT_FAILURE;
  }

  int transferred = 0;
  const int result = libusb_bulk_transfer(
    handle, BULK_IN_ENDPOINT, receive_buffer, BUFFER_COUNT * BUFFER_BYTES,
    &transferred, USB_TIMEOUT_MS);
  if (result != 0 || transferred != BUFFER_COUNT * BUFFER_BYTES)
  {
    fprintf(stderr, "Active-path bulk read failed: %s, %d bytes\n",
      libusb_error_name(result), transferred);
    return EXIT_FAILURE;
  }
  return verify_batch(receive_buffer, seed, false);
}

static int receive_reprime_batch(
  libusb_device_handle* const handle,
  uint8_t* const receive_buffer,
  const uint16_t seed)
{
  bulk_thread_args_t bulk = {
    .handle = handle,
    .buffer = receive_buffer,
    .length = REPRIME_FIRST_BYTES + (BUFFER_COUNT - 1) * BUFFER_BYTES,
    .transferred = 0,
    .result = LIBUSB_ERROR_OTHER,
  };
  pthread_t thread;
  if (pthread_create(&thread, NULL, bulk_thread, &bulk) != 0)
  {
    fprintf(stderr, "Unable to start the pending bulk transfer thread.\n");
    return EXIT_FAILURE;
  }

  const struct timespec host_head_start = {
    .tv_sec = 0,
    .tv_nsec = 2 * 1000 * 1000,
  };
  nanosleep(&host_head_start, NULL);

  const int control_status = vendor_out(
    handle, ATDTW_BATCH_REQUEST, FORCE_REPRIME_MODE, seed);
  pthread_join(thread, NULL);
  if (control_status != EXIT_SUCCESS)
  {
    return EXIT_FAILURE;
  }
  if (bulk.result != 0 || bulk.transferred != bulk.length)
  {
    fprintf(stderr, "Re-prime-path bulk read failed: %s, %d bytes\n",
      libusb_error_name(bulk.result), bulk.transferred);
    return EXIT_FAILURE;
  }
  return verify_batch(receive_buffer, seed, true);
}

static void print_telemetry(
  const char* const phase,
  const atdtw_telemetry_t* const t)
{
  printf(
    "%s telemetry:\n"
    "  batches=%" PRIu32 ", appends=%" PRIu32
      ", prime-in-progress=%" PRIu32 "\n"
    "  tripwire samples=%" PRIu32 ", retries=%" PRIu32
      ", active exits=%" PRIu32 ", re-prime exits=%" PRIu32 "\n"
    "  completion callbacks=%" PRIu32 ", dTDs retired=%" PRIu32
      ", maximum retired together=%" PRIu32 ", errors=%" PRIu32 "\n"
    "  schedule failures=%" PRIu32 ", forced-idle observed=%" PRIu32
      ", forced-idle timeouts=%" PRIu32 "\n"
    "  ownership generation=%" PRIu32 ", submitted=%" PRIu32
      ", retired=%" PRIu32 ", partial=%" PRIu32
      ", errors=%" PRIu32 ", backpressure=%" PRIu32 "\n",
    phase,
    t->batches_scheduled, t->append_calls, t->prime_in_progress_exits,
    t->tripwire_samples, t->tripwire_retries,
    t->endpoint_active_exits, t->endpoint_reprime_exits,
    t->completion_callbacks, t->descriptors_retired,
    t->maximum_retired_per_callback, t->descriptor_errors,
    t->schedule_failures, t->forced_idle_observed,
    t->forced_idle_timeouts, t->ownership_generation,
    t->ownership_submitted, t->ownership_retired,
    t->ownership_partial, t->ownership_errors,
    t->ownership_backpressure);
}

static int validate_phase(
  const atdtw_telemetry_t* const t,
  const unsigned batches,
  const bool force_reprime)
{
  const uint32_t expected_descriptors = batches * BUFFER_COUNT;
  const uint32_t expected_appends = batches * (BUFFER_COUNT - 1);
  if (t->batches_scheduled != batches
    || t->append_calls != expected_appends
    || t->descriptors_retired != expected_descriptors
    || t->schedule_failures != 0
    || t->descriptor_errors != 0)
  {
    fprintf(stderr, "Telemetry totals do not match the completed batches.\n");
    return EXIT_FAILURE;
  }
  if (t->ownership_generation == 0
    || t->ownership_submitted != expected_descriptors
    || t->ownership_retired != expected_descriptors
    || t->ownership_partial != (force_reprime ? batches : 0)
    || t->ownership_errors != 0
    || t->ownership_backpressure != 0)
  {
    fprintf(stderr, "Ownership retirement totals are inconsistent.\n");
    return EXIT_FAILURE;
  }
  for (size_t index = 0; index < BUFFER_COUNT; ++index)
  {
    const uint32_t expected_bytes =
      force_reprime && index == 0 ? REPRIME_FIRST_BYTES : BUFFER_BYTES;
    if (t->buffer_produced[index] != t->ownership_generation
      || t->buffer_submitted[index] != t->ownership_generation
      || t->buffer_retired[index] != t->ownership_generation
      || t->buffer_retired_bytes[index] != expected_bytes)
    {
      fprintf(stderr, "Buffer %zu did not retire its current generation.\n",
        index);
      return EXIT_FAILURE;
    }
  }

  if (force_reprime)
  {
    if (t->forced_idle_observed != batches
      || t->forced_idle_timeouts != 0
      || t->endpoint_reprime_exits < batches)
    {
      fprintf(stderr, "The forced endpoint-idle/re-prime branch was not proven.\n");
      return EXIT_FAILURE;
    }
  }
  else if (t->endpoint_active_exits < batches)
  {
    fprintf(stderr, "The endpoint-active ATDTW branch was not proven.\n");
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}

static int run_phase(
  libusb_device_handle* const handle,
  uint8_t* const receive_buffer,
  const char* const name,
  const unsigned batches,
  const bool force_reprime)
{
  if (vendor_out(handle, BULK_INIT_REQUEST, 0, 0) != EXIT_SUCCESS)
  {
    return EXIT_FAILURE;
  }

  for (unsigned batch = 0; batch < batches; ++batch)
  {
    const uint16_t seed = (uint16_t)(
      UINT16_C(0x6d31) + batch * UINT16_C(0x9e37));
    const int result = force_reprime
      ? receive_reprime_batch(handle, receive_buffer, seed)
      : receive_active_batch(handle, receive_buffer, seed);
    if (result != EXIT_SUCCESS)
    {
      fprintf(stderr, "%s failed at batch %u.\n", name, batch);
      const struct timespec failure_grace = {
        .tv_sec = 0,
        .tv_nsec = 5 * 1000 * 1000,
      };
      nanosleep(&failure_grace, NULL);
      atdtw_telemetry_t failure_telemetry;
      if (read_telemetry(handle, &failure_telemetry) == EXIT_SUCCESS)
      {
        print_telemetry("failure snapshot", &failure_telemetry);
      }
      return EXIT_FAILURE;
    }

    const struct timespec completion_grace = {
      .tv_sec = 0,
      .tv_nsec = 100 * 1000,
    };
    nanosleep(&completion_grace, NULL);
  }

  const struct timespec final_completion_grace = {
    .tv_sec = 0,
    .tv_nsec = 5 * 1000 * 1000,
  };
  nanosleep(&final_completion_grace, NULL);

  atdtw_telemetry_t telemetry;
  if (read_telemetry(handle, &telemetry) != EXIT_SUCCESS)
  {
    return EXIT_FAILURE;
  }
  print_telemetry(name, &telemetry);
  return validate_phase(&telemetry, batches, force_reprime);
}

static void usage(const char* const executable)
{
  fprintf(stderr,
    "Usage: %s [--active-batches N] [--reprime-batches N]"
    " [--serial TEXT]\n",
    executable);
}

static bool parse_count(const char* const text, unsigned* const result)
{
  char* end = NULL;
  errno = 0;
  const unsigned long parsed = strtoul(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0'
    || parsed == 0 || parsed > UINT32_MAX)
  {
    return false;
  }
  *result = (unsigned)parsed;
  return true;
}

int main(int argc, char** argv)
{
  unsigned active_batches = DEFAULT_ACTIVE_BATCHES;
  unsigned reprime_batches = DEFAULT_REPRIME_BATCHES;
  const char* expected_serial = NULL;

  for (int index = 1; index < argc; ++index)
  {
    if (strcmp(argv[index], "--active-batches") == 0
      && index + 1 < argc)
    {
      if (!parse_count(argv[++index], &active_batches))
      {
        usage(argv[0]);
        return EXIT_FAILURE;
      }
    }
    else if (strcmp(argv[index], "--reprime-batches") == 0
      && index + 1 < argc)
    {
      if (!parse_count(argv[++index], &reprime_batches))
      {
        usage(argv[0]);
        return EXIT_FAILURE;
      }
    }
    else if (strcmp(argv[index], "--serial") == 0 && index + 1 < argc)
    {
      expected_serial = argv[++index];
    }
    else
    {
      usage(argv[0]);
      return EXIT_FAILURE;
    }
  }

  libusb_context* context = NULL;
  const int init_result = libusb_init(&context);
  if (init_result != 0)
  {
    fprintf(stderr, "Unable to initialize libusb: %s\n",
      libusb_error_name(init_result));
    return EXIT_FAILURE;
  }

  libusb_device_handle* const handle = open_airspy(context, expected_serial);
  if (handle == NULL)
  {
    fprintf(stderr, "No matching Airspy R2/Mini was found.\n");
    libusb_exit(context);
    return EXIT_FAILURE;
  }

  const int claim_result = libusb_claim_interface(handle, 0);
  if (claim_result != 0)
  {
    fprintf(stderr, "Unable to claim Airspy interface 0: %s\n",
      libusb_error_name(claim_result));
    libusb_close(handle);
    libusb_exit(context);
    return EXIT_FAILURE;
  }

  uint8_t* const receive_buffer = malloc(BUFFER_COUNT * BUFFER_BYTES);
  int status = EXIT_FAILURE;
  if (receive_buffer != NULL
    && run_phase(handle, receive_buffer, "endpoint-active phase",
      active_batches, false) == EXIT_SUCCESS
    && run_phase(handle, receive_buffer, "forced-re-prime phase",
      reprime_batches, true) == EXIT_SUCCESS)
  {
    puts("PASS: payload integrity and both ATDTW decision branches proven.");
    status = EXIT_SUCCESS;
  }

  free(receive_buffer);
  libusb_release_interface(handle, 0);
  libusb_close(handle);
  libusb_exit(context);
  return status;
}
