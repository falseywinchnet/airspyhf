#include <libusb.h>

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
  AIRSPY_VID = 0x1d50,
  AIRSPY_PID = 0x60a1,
  AIRSPY_SPIFLASH_ERASE = 6,
  AIRSPY_SPIFLASH_WRITE = 7,
  AIRSPY_SPIFLASH_READ = 8,
  FLASH_PAGE_BYTES = 256,
  USB_TIMEOUT_MS = 5000
};

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
    if (serial[0] != '\0' && strstr(serial, expected_serial) == NULL)
    {
      libusb_close(handle);
      continue;
    }

    if (serial[0] == '\0')
    {
      printf("Opened the sole Airspy; its string descriptor is temporarily "
        "unreadable during recovery.\n");
    }
    else
    {
      printf("Opened Airspy: %s\n", serial);
    }
    selected = handle;
  }

  libusb_free_device_list(devices, 1);
  return selected;
}

static int flash_transfer(
  libusb_device_handle* const handle,
  const uint8_t request,
  const uint8_t direction,
  const uint32_t address,
  unsigned char* const data,
  const uint16_t length)
{
  const int result = libusb_control_transfer(
    handle,
    direction | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
    request,
    address >> 16,
    address & 0xffff,
    data,
    length,
    USB_TIMEOUT_MS);
  if (result != length)
  {
    fprintf(stderr, "request %u at 0x%06x, length %u failed: %s\n",
      request, address, length,
      result < 0 ? libusb_error_name(result) : "short transfer");
    return -1;
  }
  return 0;
}

static size_t first_chunk_length(
  const size_t page_index,
  const size_t page_length)
{
  if (page_length == 1)
  {
    return 1;
  }
  size_t first = 128 + page_index;
  if (first >= page_length)
  {
    first = page_length / 2;
  }
  return first;
}

static int transfer_image(
  libusb_device_handle* const handle,
  const uint8_t request,
  const uint8_t direction,
  unsigned char* const image,
  const size_t image_length)
{
  size_t address = 0;
  for (size_t page = 0; address < image_length; ++page)
  {
    const size_t remaining = image_length - address;
    const size_t page_length =
      remaining < FLASH_PAGE_BYTES ? remaining : FLASH_PAGE_BYTES;
    const size_t first = first_chunk_length(page, page_length);
    const size_t second = page_length - first;

    if (flash_transfer(handle, request, direction, address,
      image + address, first) != 0)
    {
      return -1;
    }
    address += first;
    if (second != 0)
    {
      if (flash_transfer(handle, request, direction, address,
        image + address, second) != 0)
      {
        return -1;
      }
      address += second;
    }

    if ((page & 15u) == 15u || address == image_length)
    {
      printf("%s %zu/%zu bytes\n",
        direction == LIBUSB_ENDPOINT_OUT ? "Wrote" : "Read",
        address, image_length);
    }
  }
  return 0;
}

int main(int argc, char** argv)
{
  const int verify_only =
    argc == 5 && strcmp(argv[1], "--verify-only") == 0;
  const int serial_index = verify_only ? 2 : 1;
  const int image_index = verify_only ? 4 : 3;
  if ((!verify_only && argc != 4)
    || strcmp(argv[serial_index], "--serial") != 0)
  {
    fprintf(stderr,
      "Usage: %s [--verify-only] --serial SERIAL firmware.bin\n", argv[0]);
    return EXIT_FAILURE;
  }

  FILE* const file = fopen(argv[image_index], "rb");
  if (file == NULL || fseek(file, 0, SEEK_END) != 0)
  {
    perror(argv[image_index]);
    return EXIT_FAILURE;
  }
  const long file_length = ftell(file);
  if (file_length <= 0 || file_length > 65536 || fseek(file, 0, SEEK_SET) != 0)
  {
    fprintf(stderr, "Firmware must contain 1..65536 bytes\n");
    fclose(file);
    return EXIT_FAILURE;
  }

  unsigned char* const image = malloc((size_t)file_length);
  unsigned char* const readback = malloc((size_t)file_length);
  if (image == NULL || readback == NULL
    || fread(image, 1, (size_t)file_length, file) != (size_t)file_length)
  {
    fprintf(stderr, "Could not load firmware image\n");
    fclose(file);
    free(image);
    free(readback);
    return EXIT_FAILURE;
  }
  fclose(file);

  libusb_context* context = NULL;
  if (libusb_init(&context) != 0)
  {
    free(image);
    free(readback);
    return EXIT_FAILURE;
  }
  libusb_device_handle* const handle =
    open_airspy(context, argv[serial_index + 1]);
  if (handle == NULL)
  {
    fprintf(stderr, "Target Airspy not found\n");
    libusb_exit(context);
    free(image);
    free(readback);
    return EXIT_FAILURE;
  }

  if (!verify_only)
  {
    /*
     * Recovery-only behavior. Never reset a healthy device for readback
     * qualification; on a shared hub this can disrupt unrelated endpoints.
     */
    const int reset = libusb_reset_device(handle);
    if (reset != 0)
    {
      fprintf(stderr, "USB bus reset failed: %s\n",
        libusb_error_name(reset));
      libusb_close(handle);
      libusb_exit(context);
      free(image);
      free(readback);
      return EXIT_FAILURE;
    }
  }

  int failed = 0;
  if (verify_only)
  {
    puts("Read-only qualification: no reset, erase, or write will occur.");
  }
  else
  {
    printf("Erasing first 64 KiB...\n");
    const int erase = libusb_control_transfer(
      handle,
      LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR
        | LIBUSB_RECIPIENT_DEVICE,
      AIRSPY_SPIFLASH_ERASE, 0, 0, NULL, 0, USB_TIMEOUT_MS);
    failed = erase != 0;
    if (failed)
    {
      fprintf(stderr, "Erase failed: %s\n",
        erase < 0 ? libusb_error_name(erase) : "unexpected response");
    }
    else if (transfer_image(handle, AIRSPY_SPIFLASH_WRITE,
      LIBUSB_ENDPOINT_OUT, image, (size_t)file_length) != 0)
    {
      failed = 1;
    }
  }

  if (!failed && transfer_image(handle, AIRSPY_SPIFLASH_READ,
    LIBUSB_ENDPOINT_IN, readback, (size_t)file_length) != 0)
  {
    failed = 1;
  }
  else if (!failed && memcmp(image, readback, (size_t)file_length) != 0)
  {
    size_t mismatch = 0;
    while (mismatch < (size_t)file_length
      && image[mismatch] == readback[mismatch])
    {
      ++mismatch;
    }
    fprintf(stderr, "Verification mismatch at 0x%06zx\n", mismatch);
    failed = 1;
  }
  else if (!failed)
  {
    printf("PASS: readback matches the image byte-for-byte.\n");
  }

  libusb_close(handle);
  libusb_exit(context);
  free(image);
  free(readback);
  return failed ? EXIT_FAILURE : EXIT_SUCCESS;
}
