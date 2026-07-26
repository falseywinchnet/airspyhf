#include <libusb.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
  AIRSPY_VID = 0x1d50,
  AIRSPY_PID = 0x60a1,
  AIRSPY_RESET_REQUEST = 0,
  USB_TIMEOUT_MS = 500
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

    unsigned char serial[256] = {0};
    if (descriptor.iSerialNumber != 0)
    {
      (void)libusb_get_string_descriptor_ascii(
        handle, descriptor.iSerialNumber, serial, sizeof(serial) - 1);
    }
    if (strstr((const char*)serial, expected_serial) != NULL)
    {
      printf("Rebooting Airspy %s\n", serial);
      selected = handle;
    }
    else
    {
      libusb_close(handle);
    }
  }

  libusb_free_device_list(devices, 1);
  return selected;
}

int main(int argc, char** argv)
{
  if (argc != 3 || strcmp(argv[1], "--serial") != 0)
  {
    fprintf(stderr, "Usage: %s --serial SERIAL\n", argv[0]);
    return EXIT_FAILURE;
  }

  libusb_context* context = NULL;
  if (libusb_init(&context) != 0)
  {
    return EXIT_FAILURE;
  }

  libusb_device_handle* const handle =
    open_airspy(context, argv[2]);
  if (handle == NULL)
  {
    fprintf(stderr, "Target Airspy not found\n");
    libusb_exit(context);
    return EXIT_FAILURE;
  }

  /*
   * Request zero is the firmware's target-local CPU reset handler. The device
   * normally disconnects before acknowledging, so a negative transfer result
   * is expected. Subsequent enumeration is the proof that reset completed.
   */
  const int result = libusb_control_transfer(
    handle,
    LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR
      | LIBUSB_RECIPIENT_DEVICE,
    AIRSPY_RESET_REQUEST,
    0,
    0,
    NULL,
    0,
    USB_TIMEOUT_MS);
  printf("Reset request returned %s; re-enumeration proves completion.\n",
    result < 0 ? libusb_error_name(result) : "success");

  libusb_close(handle);
  libusb_exit(context);
  return EXIT_SUCCESS;
}
