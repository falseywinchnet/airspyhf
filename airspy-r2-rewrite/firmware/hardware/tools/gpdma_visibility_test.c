#include <libusb.h>

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
  AIRSPY_VID = 0x1d50,
  AIRSPY_PID = 0x60a1,
  GPDMA_PROBE_REQUEST = 0x84,
  LOCAL_SRAM1_CANDIDATE = 1,
  LOCAL_SRAM2_CANDIDATE = 2,
  GPDMA_PASSED = 2,
  DEFAULT_ITERATIONS = 100,
  USB_TIMEOUT_MS = 5000
};

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
} gpdma_result_t;

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

static int test_candidate(
  libusb_device_handle* const handle,
  const uint16_t candidate,
  const char* const name,
  const unsigned iterations)
{
  uint32_t minimum_cycles = UINT32_MAX;
  uint32_t maximum_cycles = 0;
  uint64_t total_cycles = 0;

  for (unsigned iteration = 0; iteration < iterations; ++iteration)
  {
    gpdma_result_t result = {0};
    const uint16_t seed = (uint16_t)(
      UINT16_C(0x731d) + iteration * UINT16_C(0x9e37));
    const int transferred = libusb_control_transfer(
      handle,
      LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR
        | LIBUSB_RECIPIENT_DEVICE,
      GPDMA_PROBE_REQUEST, candidate, seed,
      (unsigned char*)&result, sizeof(result), USB_TIMEOUT_MS);
    if (transferred != sizeof(result))
    {
      fprintf(stderr, "FAIL %s iteration %u: %s\n",
        name, iteration,
        transferred < 0 ? libusb_error_name(transferred) : "short result");
      return EXIT_FAILURE;
    }

    if (result.status != GPDMA_PASSED
      || result.bytes != 16 * 1024
      || result.error_status != 0
      || result.expected_checksum != result.actual_checksum
      || result.mismatch_count != 0
      || result.first_mismatch_word != UINT32_MAX)
    {
      fprintf(stderr,
        "FAIL %s iteration %u: status=%" PRIu32
        " bytes=%" PRIu32 " error=%08" PRIx32
        " checksums=%08" PRIx32 "/%08" PRIx32
        " mismatches=%" PRIu32 " first=%" PRIu32 "\n",
        name, iteration, result.status, result.bytes, result.error_status,
        result.expected_checksum, result.actual_checksum,
        result.mismatch_count, result.first_mismatch_word);
      return EXIT_FAILURE;
    }

    if (result.cycles < minimum_cycles)
    {
      minimum_cycles = result.cycles;
    }
    if (result.cycles > maximum_cycles)
    {
      maximum_cycles = result.cycles;
    }
    total_cycles += result.cycles;
  }

  printf(
    "PASS %-12s %u copies of 16 KiB; cycles min/avg/max "
    "%" PRIu32 "/%.1f/%" PRIu32 "\n",
    name, iterations, minimum_cycles,
    (double)total_cycles / iterations, maximum_cycles);
  return EXIT_SUCCESS;
}

static void usage(const char* const executable)
{
  fprintf(stderr, "Usage: %s [--iterations N] [--serial TEXT]\n",
    executable);
}

int main(int argc, char** argv)
{
  unsigned iterations = DEFAULT_ITERATIONS;
  const char* expected_serial = NULL;

  for (int index = 1; index < argc; ++index)
  {
    if (strcmp(argv[index], "--iterations") == 0 && index + 1 < argc)
    {
      char* end = NULL;
      errno = 0;
      const unsigned long parsed = strtoul(argv[++index], &end, 10);
      if (errno != 0 || end == argv[index] || *end != '\0'
        || parsed == 0 || parsed > UINT32_MAX)
      {
        usage(argv[0]);
        return EXIT_FAILURE;
      }
      iterations = (unsigned)parsed;
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

  const int status =
    test_candidate(
      handle, LOCAL_SRAM1_CANDIDATE, "local SRAM1", iterations)
      == EXIT_SUCCESS
    && test_candidate(
      handle, LOCAL_SRAM2_CANDIDATE, "local SRAM2", iterations)
      == EXIT_SUCCESS
    ? EXIT_SUCCESS
    : EXIT_FAILURE;

  libusb_close(handle);
  libusb_exit(context);
  return status;
}
