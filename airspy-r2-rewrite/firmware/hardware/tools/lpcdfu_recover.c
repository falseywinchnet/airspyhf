#include <libusb.h>

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

enum
{
  LPC_VID = 0x1fc9,
  LPC_PID = 0x000c,
  DFU_DNLOAD = 1,
  DFU_UPLOAD = 2,
  DFU_GETSTATUS = 3,
  DFU_GETSTATE = 5,
  DFU_IDLE = 2,
  DFU_DNLOAD_IDLE = 5,
  LPC_COMMAND_PREPARE = 0,
  LPC_COMMAND_ERASE_REGION = 2,
  LPC_COMMAND_PROGRAM = 3,
  LPC_COMMAND_UPLOAD = 4,
  LPC_OPERATION_COMPLETE = 0,
  LPC_OPERATION_UPLOAD_READY = 6,
  LPC_OPERATION_PROGRAM_READY = 13,
  USB_TIMEOUT_MS = 5000,
  TRANSFER_BYTES = 2048,
  OPERATION_POLL_LIMIT = 10000
};

static const uint32_t LPC_SPIFI_LOGICAL_BASE = 0x80000000u;

typedef struct
{
  uint32_t command;
  uint32_t state;
  uint32_t log_length;
  uint32_t transfer_size;
  char log[65];
} lpc_operation_t;

static int control(
  libusb_device_handle* const handle,
  const uint8_t direction,
  const uint8_t request,
  const uint16_t value,
  unsigned char* const data,
  const uint16_t length)
{
  return libusb_control_transfer(
    handle,
    direction | LIBUSB_REQUEST_TYPE_CLASS | LIBUSB_RECIPIENT_INTERFACE,
    request, value, 0, data, length, USB_TIMEOUT_MS);
}

static libusb_device_handle* open_loader(libusb_context* const context)
{
  libusb_device_handle* const handle =
    libusb_open_device_with_vid_pid(context, LPC_VID, LPC_PID);
  if (handle == NULL)
  {
    return NULL;
  }
  if (libusb_claim_interface(handle, 0) != 0)
  {
    libusb_close(handle);
    return NULL;
  }
  return handle;
}

static uint32_t load_u32(const unsigned char* const data)
{
  return (uint32_t)data[0]
    | (uint32_t)data[1] << 8
    | (uint32_t)data[2] << 16
    | (uint32_t)data[3] << 24;
}

static void store_u32(unsigned char* const data, const uint32_t value)
{
  data[0] = value;
  data[1] = value >> 8;
  data[2] = value >> 16;
  data[3] = value >> 24;
}

static int wait_dfu_state(
  libusb_device_handle* const handle,
  const uint8_t expected)
{
  for (unsigned int attempt = 0; attempt < 100; ++attempt)
  {
    unsigned char status[6] = {0};
    const int result =
      control(handle, LIBUSB_ENDPOINT_IN, DFU_GETSTATUS, 0,
        status, sizeof(status));
    if (result != sizeof(status))
    {
      fprintf(stderr, "DFU status transfer failed: %s\n",
        libusb_error_name(result));
      return -1;
    }
    if (status[0] != 0)
    {
      fprintf(stderr, "DFU reported error status %u in state %u\n",
        status[0], status[4]);
      return -1;
    }
    if (status[4] == expected)
    {
      return 0;
    }
    const uint32_t delay_ms =
      (uint32_t)status[1] | (uint32_t)status[2] << 8
      | (uint32_t)status[3] << 16;
    usleep((delay_ms == 0 ? 1 : delay_ms) * 1000);
  }
  fprintf(stderr, "Timed out waiting for DFU state %u\n", expected);
  return -1;
}

static int read_operation(
  libusb_device_handle* const handle,
  lpc_operation_t* const operation)
{
  unsigned char response[80] = {0};
  const int result =
    control(handle, LIBUSB_ENDPOINT_IN, DFU_UPLOAD, 0,
      response, sizeof(response));
  if (result < 16 || result > (int)sizeof(response))
  {
    fprintf(stderr, "Operation-status transfer returned %d bytes\n", result);
    return -1;
  }
  operation->command = load_u32(response);
  operation->state = load_u32(response + 4);
  operation->log_length = load_u32(response + 8);
  operation->transfer_size = load_u32(response + 12);
  const size_t available_log = (size_t)result - 16;
  const size_t log_length =
    operation->log_length < available_log
      ? operation->log_length : available_log;
  memcpy(operation->log, response + 16, log_length);
  operation->log[log_length] = '\0';
  return 0;
}

static int wait_operation(
  libusb_device_handle* const handle,
  const uint32_t expected_state,
  const uint32_t expected_command)
{
  for (unsigned int attempt = 0; attempt < OPERATION_POLL_LIMIT; ++attempt)
  {
    lpc_operation_t operation;
    if (read_operation(handle, &operation) != 0)
    {
      return -1;
    }
    if (operation.state == expected_state
      && operation.command == expected_command)
    {
      if (operation.transfer_size != TRANSFER_BYTES)
      {
        fprintf(stderr,
          "Loader transfer size is %u rather than the expected %u\n",
          operation.transfer_size, TRANSFER_BYTES);
        return -1;
      }
      return 0;
    }
    if ((operation.state >= 1 && operation.state <= 4)
      || operation.state == 16)
    {
      fprintf(stderr,
        "Loader command %u stopped in error state %u%s%s\n",
        operation.command, operation.state,
        operation.log[0] == '\0' ? "" : ": ",
        operation.log);
      return -1;
    }
    usleep(1000);
  }
  fprintf(stderr,
    "Timed out waiting for loader operation state %u, command %u\n",
    expected_state, expected_command);
  return -1;
}

static int send_command(
  libusb_device_handle* const handle,
  const uint32_t command,
  const uint32_t address,
  const uint32_t length)
{
  unsigned char packet[16];
  store_u32(packet, command);
  store_u32(packet + 4, address);
  store_u32(packet + 8, length);
  store_u32(packet + 12, 0x1843010au);
  if (control(handle, LIBUSB_ENDPOINT_OUT, DFU_DNLOAD, 0,
      packet, sizeof(packet)) != sizeof(packet)
    || wait_dfu_state(handle, DFU_DNLOAD_IDLE) != 0
    || control(handle, LIBUSB_ENDPOINT_OUT, DFU_DNLOAD, 1, NULL, 0) != 0
    || wait_dfu_state(handle, DFU_IDLE) != 0)
  {
    fprintf(stderr, "Loader command %u could not be sent\n", command);
    return -1;
  }
  return 0;
}

static int send_program_chunk(
  libusb_device_handle* const handle,
  const unsigned char* const data,
  const uint16_t length)
{
  if (control(handle, LIBUSB_ENDPOINT_OUT, DFU_DNLOAD, 0,
      (unsigned char*)data, length) != length
    || wait_dfu_state(handle, DFU_DNLOAD_IDLE) != 0
    || control(handle, LIBUSB_ENDPOINT_OUT, DFU_DNLOAD, 1, NULL, 0) != 0
    || wait_dfu_state(handle, DFU_IDLE) != 0)
  {
    fprintf(stderr, "Program data transfer failed\n");
    return -1;
  }
  return 0;
}

static int read_file(
  const char* const path,
  unsigned char** const data_out,
  size_t* const length_out)
{
  FILE* const file = fopen(path, "rb");
  if (file == NULL)
  {
    fprintf(stderr, "Could not open %s: %s\n", path, strerror(errno));
    return -1;
  }
  if (fseek(file, 0, SEEK_END) != 0)
  {
    fclose(file);
    return -1;
  }
  const long file_length = ftell(file);
  if (file_length <= 0 || file_length > UINT32_MAX
    || fseek(file, 0, SEEK_SET) != 0)
  {
    fprintf(stderr, "Invalid image length for %s\n", path);
    fclose(file);
    return -1;
  }
  unsigned char* const data = malloc((size_t)file_length);
  if (data == NULL
    || fread(data, 1, (size_t)file_length, file) != (size_t)file_length)
  {
    fprintf(stderr, "Could not read all of %s\n", path);
    free(data);
    fclose(file);
    return -1;
  }
  fclose(file);
  *data_out = data;
  *length_out = (size_t)file_length;
  return 0;
}

static int erase_region(
  libusb_device_handle* const handle,
  const uint32_t length)
{
  const uint32_t erase_length = (length + 4095u) & ~4095u;
  printf("Erasing %u-byte application region (%u bytes of image data)...\n",
    erase_length, length);
  if (send_command(handle, LPC_COMMAND_ERASE_REGION,
      LPC_SPIFI_LOGICAL_BASE, erase_length) != 0
    || wait_operation(handle, LPC_OPERATION_COMPLETE,
      LPC_COMMAND_ERASE_REGION) != 0)
  {
    return -1;
  }
  puts("Erase complete.");
  return 0;
}

static int program_region(
  libusb_device_handle* const handle,
  const unsigned char* const image,
  const size_t length)
{
  puts("Programming corrected firmware...");
  if (send_command(handle, LPC_COMMAND_PROGRAM,
      LPC_SPIFI_LOGICAL_BASE, (uint32_t)length) != 0)
  {
    return -1;
  }
  size_t offset = 0;
  while (offset < length)
  {
    const size_t remaining = length - offset;
    const uint16_t chunk =
      remaining > TRANSFER_BYTES ? TRANSFER_BYTES : (uint16_t)remaining;
    if (wait_operation(handle, LPC_OPERATION_PROGRAM_READY,
        LPC_COMMAND_PROGRAM) != 0
      || send_program_chunk(handle, image + offset, chunk) != 0)
    {
      fprintf(stderr, "Programming stopped at byte %zu\n", offset);
      return -1;
    }
    offset += chunk;
    printf("\rProgrammed %zu / %zu bytes", offset, length);
    fflush(stdout);
  }
  putchar('\n');
  if (wait_operation(handle, LPC_OPERATION_COMPLETE,
      LPC_COMMAND_PROGRAM) != 0)
  {
    return -1;
  }
  puts("Programming complete.");
  return 0;
}

static int upload_region(
  libusb_device_handle* const handle,
  unsigned char* const data,
  const size_t length)
{
  puts("Reading the programmed image back...");
  if (send_command(handle, LPC_COMMAND_UPLOAD,
      LPC_SPIFI_LOGICAL_BASE, (uint32_t)length) != 0)
  {
    return -1;
  }
  size_t offset = 0;
  while (offset < length)
  {
    const size_t remaining = length - offset;
    const uint16_t chunk =
      remaining > TRANSFER_BYTES ? TRANSFER_BYTES : (uint16_t)remaining;
    if (wait_operation(handle, LPC_OPERATION_UPLOAD_READY,
        LPC_COMMAND_UPLOAD) != 0)
    {
      fprintf(stderr, "Readback stopped at byte %zu\n", offset);
      return -1;
    }
    const int result =
      control(handle, LIBUSB_ENDPOINT_IN, DFU_UPLOAD, 0,
        data + offset, chunk);
    if (result != chunk)
    {
      fprintf(stderr,
        "Readback at byte %zu returned %d rather than %u bytes\n",
        offset, result, chunk);
      return -1;
    }
    offset += chunk;
    printf("\rRead back %zu / %zu bytes", offset, length);
    fflush(stdout);
  }
  putchar('\n');
  if (wait_operation(handle, LPC_OPERATION_COMPLETE,
      LPC_COMMAND_UPLOAD) != 0)
  {
    return -1;
  }
  return 0;
}

static void print_current_status(libusb_device_handle* const handle)
{
  unsigned char state = 0xff;
  const int state_result =
    control(handle, LIBUSB_ENDPOINT_IN, DFU_GETSTATE, 0, &state, 1);
  lpc_operation_t operation;
  const int operation_result = read_operation(handle, &operation);
  printf("DFU state: result=%d state=%u\n", state_result, state);
  if (operation_result == 0)
  {
    printf("Loader operation: command=%u state=%u log=%u transfer=%u%s%s\n",
      operation.command, operation.state, operation.log_length,
      operation.transfer_size,
      operation.log[0] == '\0' ? "" : " message=",
      operation.log);
  }
}

int main(int argc, char** argv)
{
  const int status_only =
    argc == 2 && strcmp(argv[1], "--status") == 0;
  if (!status_only
    && (argc != 3 || strcmp(argv[1], "--write-verify") != 0))
  {
    fprintf(stderr,
      "Usage: %s --status | --write-verify firmware.bin\n", argv[0]);
    return EXIT_FAILURE;
  }

  unsigned char* image = NULL;
  size_t image_length = 0;
  if (!status_only && read_file(argv[2], &image, &image_length) != 0)
  {
    return EXIT_FAILURE;
  }
  if (!status_only)
  {
    printf("Image: %s (%zu bytes)\n", argv[2], image_length);
  }

  libusb_context* context = NULL;
  if (libusb_init(&context) != 0)
  {
    free(image);
    return EXIT_FAILURE;
  }
  libusb_device_handle* const handle = open_loader(context);
  if (handle == NULL)
  {
    fprintf(stderr, "LPC DFU recovery loader was not found\n");
    libusb_exit(context);
    free(image);
    return EXIT_FAILURE;
  }

  if (status_only)
  {
    print_current_status(handle);
    libusb_release_interface(handle, 0);
    libusb_close(handle);
    libusb_exit(context);
    return EXIT_SUCCESS;
  }

  int result = EXIT_FAILURE;
  unsigned char* readback = malloc(image_length);
  if (readback == NULL)
  {
    fprintf(stderr, "Could not allocate readback buffer\n");
    goto cleanup;
  }

  puts("Preparing the RAM-resident recovery loader...");
  if (send_command(handle, LPC_COMMAND_PREPARE, 0, 0) != 0
    || wait_operation(handle, LPC_OPERATION_COMPLETE,
      LPC_COMMAND_PREPARE) != 0
    || erase_region(handle, (uint32_t)image_length) != 0
    || program_region(handle, image, image_length) != 0
    || upload_region(handle, readback, image_length) != 0)
  {
    print_current_status(handle);
    goto cleanup;
  }

  if (memcmp(image, readback, image_length) != 0)
  {
    size_t mismatch = 0;
    while (mismatch < image_length
      && image[mismatch] == readback[mismatch])
    {
      ++mismatch;
    }
    fprintf(stderr,
      "VERIFY FAILED at byte %zu: image=%02x flash=%02x\n",
      mismatch, image[mismatch], readback[mismatch]);
    goto cleanup;
  }

  printf("VERIFY PASS: all %zu bytes match exactly.\n", image_length);
  puts("The loader remains active; no USB reset was requested.");
  result = EXIT_SUCCESS;

cleanup:
  free(readback);
  libusb_release_interface(handle, 0);
  libusb_close(handle);
  libusb_exit(context);
  free(image);
  return result;
}
