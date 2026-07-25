/*
 * Reads the ADCHS FIFO overflow counter added by the instrumentation patch to
 * upstream airspyone_firmware (vendor command 28).
 *
 * Usage:
 *   fifo_ovf            print the counter
 *   fifo_ovf -c         print the counter, then clear it
 *
 * This talks to the device with a plain control transfer and does not open a
 * stream, so it can be run while another application is receiving.
 */
#include <libusb.h>
#include <stdio.h>
#include <string.h>

#define AIRSPY_VID 0x1d50
#define AIRSPY_PID 0x60a1
#define AIRSPY_GET_ADCHS_FIFO_OVF 28

int main(int argc, char **argv)
{
  int clear = (argc > 1 && strcmp(argv[1], "-c") == 0);
  libusb_context *ctx = NULL;
  libusb_device_handle *dev;
  unsigned char buf[4] = {0};
  int r;

  if (libusb_init(&ctx) != 0) {
    fprintf(stderr, "libusb_init failed\n");
    return 1;
  }

  /*
   * More than one Airspy may be attached, and only the one running the
   * instrumented firmware answers command 28. Try each in turn rather than
   * taking whichever enumerates first; the others stall, which is expected.
   */
  {
    libusb_device **list;
    ssize_t n = libusb_get_device_list(ctx, &list);
    ssize_t i;
    int found = 0;

    r = LIBUSB_ERROR_NO_DEVICE;
    for (i = 0; i < n && !found; i++) {
      struct libusb_device_descriptor desc;
      if (libusb_get_device_descriptor(list[i], &desc) != 0)
        continue;
      if (desc.idVendor != AIRSPY_VID || desc.idProduct != AIRSPY_PID)
        continue;
      if (libusb_open(list[i], &dev) != 0)
        continue;

      r = libusb_control_transfer(dev,
            LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR
              | LIBUSB_RECIPIENT_DEVICE,
            AIRSPY_GET_ADCHS_FIFO_OVF,
            clear ? 1 : 0, /* wValue != 0 clears after reading */
            0, buf, sizeof(buf), 1000);

      if (r == (int)sizeof(buf))
        found = 1;
      else
        libusb_close(dev);
    }
    libusb_free_device_list(list, 1);

    if (!found) {
      fprintf(stderr, "no Airspy answered command %d: %s\n",
              AIRSPY_GET_ADCHS_FIFO_OVF,
              r < 0 ? libusb_error_name(r) : "short read");
      fprintf(stderr, "(instrumented firmware flashed, and power cycled?)\n");
      libusb_exit(ctx);
      return 1;
    }
  }

  printf("adchs_fifo_ovf = %u%s\n",
         (unsigned)(buf[0] | (buf[1] << 8) | (buf[2] << 16) | (buf[3] << 24)),
         clear ? " (cleared)" : "");

  libusb_close(dev);
  libusb_exit(ctx);
  return 0;
}
