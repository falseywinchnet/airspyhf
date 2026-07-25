#include <libusb.h>

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

enum {
  AIRSPY_VID = 0x1d50,
  AIRSPY_PID = 0x60a1,
  MEMORY_READ_REQUEST = 0x80,
  RESET_REQUEST = 0x00,
  BUFFER_BYTES = 16 * 1024,
  DEFAULT_ITERATIONS = 200,
  USB_TIMEOUT_MS = 3000
};

typedef struct {
  const char* name;
  uint16_t id;
  uintptr_t address;
  bool experimental;
} candidate_t;

static const candidate_t candidates[] = {
  {"AHB SRAM reference", 0, 0x20004000u, false},
  {"local SRAM1", 1, 0x10018000u, false},
  {"local SRAM2", 2, 0x10084000u, false},
  {"M0-subsystem SRAM", 3, 0x18000800u, true},
};

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

static double monotonic_seconds(void)
{
  struct timespec now;
  if (clock_gettime(CLOCK_MONOTONIC, &now) != 0)
  {
    return 0.0;
  }
  return (double)now.tv_sec + (double)now.tv_nsec / 1000000000.0;
}

static libusb_device_handle* open_airspy(
  libusb_context* const context,
  const char* const expected_serial)
{
  libusb_device** devices = NULL;
  const ssize_t count = libusb_get_device_list(context, &devices);
  if (count < 0)
  {
    fprintf(stderr, "Unable to enumerate USB devices: %s\n",
      libusb_error_name((int)count));
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

    printf("Opened Airspy: %s at USB bus %u address %u, speed %d\n",
      serial[0] != '\0' ? serial : "(serial unavailable)",
      libusb_get_bus_number(devices[index]),
      libusb_get_device_address(devices[index]),
      libusb_get_device_speed(devices[index]));
    selected = handle;
  }

  libusb_free_device_list(devices, 1);
  return selected;
}

static int reset_airspy(libusb_device_handle* const handle)
{
  const int result = libusb_control_transfer(
    handle,
    LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR
      | LIBUSB_RECIPIENT_DEVICE,
    RESET_REQUEST, 0, 0, NULL, 0, USB_TIMEOUT_MS);

  /*
   * The firmware resets before acknowledging the request, so a disconnect,
   * I/O error, or timeout is an expected successful outcome.
   */
  if (result == 0 || result == LIBUSB_ERROR_IO
    || result == LIBUSB_ERROR_NO_DEVICE || result == LIBUSB_ERROR_TIMEOUT)
  {
    puts("Reset request sent; wait for the Airspy to re-enumerate.");
    return EXIT_SUCCESS;
  }

  fprintf(stderr, "Reset request failed: %s\n", libusb_error_name(result));
  return EXIT_FAILURE;
}

static int test_candidate(
  libusb_device_handle* const handle,
  const candidate_t* const candidate,
  const unsigned iterations)
{
  uint8_t* const buffer = malloc(BUFFER_BYTES);
  if (buffer == NULL)
  {
    fprintf(stderr, "Unable to allocate the host receive buffer.\n");
    return EXIT_FAILURE;
  }

  const double started = monotonic_seconds();
  uint64_t bytes_read = 0;
  for (unsigned iteration = 0; iteration < iterations; ++iteration)
  {
    const uint16_t seed = (uint16_t)(
      UINT16_C(0x51a7) + iteration * UINT16_C(0x9e37));
    memset(buffer, 0xcc, BUFFER_BYTES);

    const int transferred = libusb_control_transfer(
      handle,
      LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR
        | LIBUSB_RECIPIENT_DEVICE,
      MEMORY_READ_REQUEST, candidate->id, seed, buffer, BUFFER_BYTES,
      USB_TIMEOUT_MS);
    if (transferred != BUFFER_BYTES)
    {
      fprintf(stderr,
        "FAIL %-20s iteration %u: received %d of %d bytes (%s)\n",
        candidate->name, iteration, transferred, BUFFER_BYTES,
        transferred < 0 ? libusb_error_name(transferred) : "short transfer");
      free(buffer);
      return EXIT_FAILURE;
    }

    for (uint32_t word_index = 0;
      word_index < BUFFER_BYTES / sizeof(uint32_t); ++word_index)
    {
      const uint32_t actual = load_le32(buffer + word_index * 4);
      const uint32_t expected = expected_word(
        candidate->address, seed, word_index);
      if (actual != expected)
      {
        fprintf(stderr,
          "FAIL %-20s iteration %u word %" PRIu32
          " (address 0x%08" PRIxPTR "): expected %08" PRIx32
          ", got %08" PRIx32 "\n",
          candidate->name, iteration, word_index,
          candidate->address + word_index * 4, expected, actual);
        free(buffer);
        return EXIT_FAILURE;
      }
    }
    bytes_read += BUFFER_BYTES;
  }

  const double elapsed = monotonic_seconds() - started;
  printf("PASS %-20s %u transfers, %.2f MiB, %.2f MiB/s\n",
    candidate->name, iterations,
    (double)bytes_read / (1024.0 * 1024.0),
    elapsed > 0.0
      ? (double)bytes_read / (1024.0 * 1024.0) / elapsed
      : 0.0);
  free(buffer);
  return EXIT_SUCCESS;
}

static void usage(const char* const executable)
{
  fprintf(stderr,
    "Usage: %s [--iterations N] [--serial TEXT] [--include-m0sub]"
    " [--reset]\n",
    executable);
}

int main(int argc, char** argv)
{
  unsigned iterations = DEFAULT_ITERATIONS;
  const char* expected_serial = NULL;
  bool include_m0sub = false;
  bool reset_only = false;

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
    else if (strcmp(argv[index], "--include-m0sub") == 0)
    {
      include_m0sub = true;
    }
    else if (strcmp(argv[index], "--reset") == 0)
    {
      reset_only = true;
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

  int status = EXIT_SUCCESS;
  if (reset_only)
  {
    status = reset_airspy(handle);
  }
  else
  {
    for (size_t index = 0;
      index < sizeof(candidates) / sizeof(candidates[0]); ++index)
    {
      if (candidates[index].experimental && !include_m0sub)
      {
        continue;
      }
      if (test_candidate(handle, &candidates[index], iterations)
        != EXIT_SUCCESS)
      {
        status = EXIT_FAILURE;
        break;
      }
    }
  }

  libusb_close(handle);
  libusb_exit(context);
  return status;
}
