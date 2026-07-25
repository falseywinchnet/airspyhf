/*
 * x86_64 Windows airspy.dll shim for SDR# under Wine.
 * USB remains native: this DLL forwards the libairspy ABI to helper.c.
 */
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "airspy.h"
#include "proto.h"

struct airspy_device {
    SOCKET ctrl;
    CRITICAL_SECTION lock;
    uint64_t id;

    airspy_sample_block_cb_fn callback;
    void *callback_ctx;
    SOCKET data;
    HANDLE reader;
    volatile LONG streaming;
    uint8_t *buffer;
    uint32_t buffer_capacity;
};

static volatile LONG winsock_started;

static void ensure_winsock(void)
{
    if (InterlockedCompareExchange(&winsock_started, 1, 0) == 0) {
        WSADATA data;
        WSAStartup(MAKEWORD(2, 2), &data);
    }
}

static int bridge_port(void)
{
    char text[16];
    DWORD length = GetEnvironmentVariableA(AOB_PORT_ENV, text, sizeof(text));
    if (length > 0 && length < sizeof(text)) {
        int port = atoi(text);
        if (port > 0) return port;
    }
    return AOB_DEFAULT_PORT;
}

static int send_full(SOCKET socket, const void *data, uint32_t length)
{
    const char *cursor = (const char *)data;
    while (length) {
        int chunk = length > 0x40000000u ? 0x40000000 : (int)length;
        int sent = send(socket, cursor, chunk, 0);
        if (sent <= 0) return -1;
        cursor += sent;
        length -= (uint32_t)sent;
    }
    return 0;
}

static int receive_full(SOCKET socket, void *data, uint32_t length)
{
    char *cursor = (char *)data;
    while (length) {
        int chunk = length > 0x40000000u ? 0x40000000 : (int)length;
        int received = recv(socket, cursor, chunk, 0);
        if (received <= 0) return -1;
        cursor += received;
        length -= (uint32_t)received;
    }
    return 0;
}

static SOCKET connect_channel(uint32_t channel)
{
    ensure_winsock();
    SOCKET socket = WSASocketA(AF_INET, SOCK_STREAM, IPPROTO_TCP, NULL, 0, 0);
    if (socket == INVALID_SOCKET) return INVALID_SOCKET;

    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_port = htons((u_short)bridge_port());
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (connect(socket, (struct sockaddr *)&address, sizeof(address)) != 0) {
        closesocket(socket);
        return INVALID_SOCKET;
    }

    int one = 1;
    int buffer_size = 4 * 1024 * 1024;
    setsockopt(socket, IPPROTO_TCP, TCP_NODELAY, (const char *)&one, sizeof(one));
    setsockopt(socket, SOL_SOCKET, SO_RCVBUF, (const char *)&buffer_size, sizeof(buffer_size));
    if (send_full(socket, &channel, sizeof(channel)) != 0) {
        closesocket(socket);
        return INVALID_SOCKET;
    }
    return socket;
}

static int rpc_on(SOCKET socket, uint32_t op, uint64_t device_id,
                  const void *input, uint32_t input_length,
                  void *output, uint32_t output_capacity)
{
    aob_req_hdr request = { op, device_id, input_length };
    if (send_full(socket, &request, sizeof(request)) != 0) return AIRSPY_ERROR_OTHER;
    if (input_length && send_full(socket, input, input_length) != 0) return AIRSPY_ERROR_OTHER;

    aob_resp_hdr response;
    if (receive_full(socket, &response, sizeof(response)) != 0) return AIRSPY_ERROR_OTHER;

    uint32_t copied = response.out_len < output_capacity ? response.out_len : output_capacity;
    if (copied && receive_full(socket, output, copied) != 0) return AIRSPY_ERROR_OTHER;
    uint32_t remaining = response.out_len - copied;
    while (remaining) {
        uint8_t discard[512];
        uint32_t part = remaining < sizeof(discard) ? remaining : sizeof(discard);
        if (receive_full(socket, discard, part) != 0) return AIRSPY_ERROR_OTHER;
        remaining -= part;
    }
    return response.ret;
}

static int global_rpc(uint32_t op, const void *input, uint32_t input_length,
                      void *output, uint32_t output_capacity)
{
    SOCKET socket = connect_channel(AOB_CHANNEL_CONTROL);
    if (socket == INVALID_SOCKET) return AIRSPY_ERROR_OTHER;
    int result = rpc_on(socket, op, 0, input, input_length, output, output_capacity);
    closesocket(socket);
    return result;
}

static int device_rpc(struct airspy_device *device, uint32_t op,
                      const void *input, uint32_t input_length,
                      void *output, uint32_t output_capacity)
{
    if (!device) return AIRSPY_ERROR_INVALID_PARAM;
    EnterCriticalSection(&device->lock);
    int result = rpc_on(device->ctrl, op, device->id,
                        input, input_length, output, output_capacity);
    LeaveCriticalSection(&device->lock);
    return result;
}

static DWORD WINAPI sample_reader(void *argument)
{
    struct airspy_device *device = (struct airspy_device *)argument;
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);

    while (InterlockedCompareExchange(&device->streaming, 1, 1)) {
        aob_data_hdr header;
        if (receive_full(device->data, &header, sizeof(header)) != 0) break;
        if (header.magic != AOB_DATA_FRAME_MAGIC ||
            header.sample_type >= AIRSPY_SAMPLE_END ||
            header.payload_bytes > 16u * 1024u * 1024u) break;

        if (header.payload_bytes > device->buffer_capacity) {
            uint8_t *larger = (uint8_t *)realloc(device->buffer, header.payload_bytes);
            if (!larger) break;
            device->buffer = larger;
            device->buffer_capacity = header.payload_bytes;
        }
        if (header.payload_bytes &&
            receive_full(device->data, device->buffer, header.payload_bytes) != 0) break;

        airspy_transfer_t transfer;
        transfer.device = device;
        transfer.ctx = device->callback_ctx;
        transfer.samples = device->buffer;
        transfer.sample_count = (int)header.sample_count;
        transfer.dropped_samples = header.dropped_samples;
        transfer.sample_type = (enum airspy_sample_type)header.sample_type;
        if (device->callback && device->callback(&transfer) != 0) {
            InterlockedExchange(&device->streaming, 0);
            break;
        }
    }
    InterlockedExchange(&device->streaming, 0);
    return 0;
}

static int open_common(struct airspy_device **output, uint32_t op,
                       const void *input, uint32_t input_length)
{
    if (!output) return AIRSPY_ERROR_INVALID_PARAM;
    *output = NULL;
    SOCKET socket = connect_channel(AOB_CHANNEL_CONTROL);
    if (socket == INVALID_SOCKET) return AIRSPY_ERROR_OTHER;

    uint64_t id = 0;
    int result = rpc_on(socket, op, 0, input, input_length, &id, sizeof(id));
    if (result != AIRSPY_SUCCESS) {
        closesocket(socket);
        return result;
    }

    struct airspy_device *device = (struct airspy_device *)calloc(1, sizeof(*device));
    if (!device) {
        closesocket(socket);
        return AIRSPY_ERROR_NO_MEM;
    }
    device->ctrl = socket;
    device->data = INVALID_SOCKET;
    device->id = id;
    InitializeCriticalSection(&device->lock);
    *output = device;
    return AIRSPY_SUCCESS;
}

void ADDCALL airspy_lib_version(airspy_lib_version_t *version)
{
    if (!version) return;
    if (global_rpc(AOB_OP_LIB_VERSION, NULL, 0, version, sizeof(*version)) != AIRSPY_SUCCESS) {
        version->major_version = AIRSPY_VER_MAJOR;
        version->minor_version = AIRSPY_VER_MINOR;
        version->revision = AIRSPY_VER_REVISION;
    }
}

int ADDCALL airspy_init(void)
{
    return global_rpc(AOB_OP_INIT, NULL, 0, NULL, 0);
}

int ADDCALL airspy_exit(void)
{
    return global_rpc(AOB_OP_EXIT, NULL, 0, NULL, 0);
}

int ADDCALL airspy_list_devices(uint64_t *serials, int count)
{
    if (count < 0) return AIRSPY_ERROR_INVALID_PARAM;
    return global_rpc(AOB_OP_LIST_DEVICES, &count, sizeof(count),
                      serials, serials ? (uint32_t)count * sizeof(*serials) : 0);
}

int ADDCALL airspy_open(struct airspy_device **device)
{
    return open_common(device, AOB_OP_OPEN, NULL, 0);
}

int ADDCALL airspy_open_sn(struct airspy_device **device, uint64_t serial)
{
    return open_common(device, AOB_OP_OPEN_SN, &serial, sizeof(serial));
}

int ADDCALL airspy_open_fd(struct airspy_device **device, int fd)
{
    (void)device;
    (void)fd;
    return AIRSPY_ERROR_UNSUPPORTED;
}

int ADDCALL airspy_stop_rx(struct airspy_device *device)
{
    if (!device) return AIRSPY_ERROR_INVALID_PARAM;
    int result = device_rpc(device, AOB_OP_STOP_RX, NULL, 0, NULL, 0);
    InterlockedExchange(&device->streaming, 0);
    if (device->data != INVALID_SOCKET) {
        shutdown(device->data, SD_BOTH);
        closesocket(device->data);
        device->data = INVALID_SOCKET;
    }
    if (device->reader) {
        WaitForSingleObject(device->reader, 3000);
        CloseHandle(device->reader);
        device->reader = NULL;
    }
    return result;
}

int ADDCALL airspy_close(struct airspy_device *device)
{
    if (!device) return AIRSPY_ERROR_INVALID_PARAM;
    if (device->reader || InterlockedCompareExchange(&device->streaming, 1, 1))
        airspy_stop_rx(device);
    int result = device_rpc(device, AOB_OP_CLOSE, NULL, 0, NULL, 0);
    closesocket(device->ctrl);
    DeleteCriticalSection(&device->lock);
    free(device->buffer);
    free(device);
    return result;
}

int ADDCALL airspy_start_rx(struct airspy_device *device,
                           airspy_sample_block_cb_fn callback, void *context)
{
    if (!device || !callback) return AIRSPY_ERROR_INVALID_PARAM;
    device->callback = callback;
    device->callback_ctx = context;

    device->data = connect_channel(AOB_CHANNEL_DATA);
    if (device->data == INVALID_SOCKET) return AIRSPY_ERROR_OTHER;
    aob_data_hello hello = { device->id };
    if (send_full(device->data, &hello, sizeof(hello)) != 0) {
        closesocket(device->data);
        device->data = INVALID_SOCKET;
        return AIRSPY_ERROR_OTHER;
    }

    int result = device_rpc(device, AOB_OP_START_RX, NULL, 0, NULL, 0);
    if (result != AIRSPY_SUCCESS) {
        closesocket(device->data);
        device->data = INVALID_SOCKET;
        return result;
    }

    InterlockedExchange(&device->streaming, 1);
    device->reader = CreateThread(NULL, 0, sample_reader, device, 0, NULL);
    if (!device->reader) {
        airspy_stop_rx(device);
        return AIRSPY_ERROR_THREAD;
    }
    return AIRSPY_SUCCESS;
}

int ADDCALL airspy_is_streaming(struct airspy_device *device)
{
    return device_rpc(device, AOB_OP_IS_STREAMING, NULL, 0, NULL, 0);
}

int ADDCALL airspy_get_samplerates(struct airspy_device *device, uint32_t *buffer,
                                  const uint32_t length)
{
    if (!buffer) return AIRSPY_ERROR_INVALID_PARAM;
    uint32_t output_count = length ? length : 1;
    return device_rpc(device, AOB_OP_GET_SAMPLERATES, &length, sizeof(length),
                      buffer, output_count * sizeof(*buffer));
}

#define SET_U32(function_name, opcode) \
    int ADDCALL function_name(struct airspy_device *device, uint32_t value) { \
        return device_rpc(device, opcode, &value, sizeof(value), NULL, 0); \
    }

#define SET_U8(function_name, opcode) \
    int ADDCALL function_name(struct airspy_device *device, uint8_t value) { \
        return device_rpc(device, opcode, &value, sizeof(value), NULL, 0); \
    }

SET_U32(airspy_set_samplerate, AOB_OP_SET_SAMPLERATE)
SET_U32(airspy_set_freq, AOB_OP_SET_FREQ)
SET_U8(airspy_set_lna_gain, AOB_OP_SET_LNA_GAIN)
SET_U8(airspy_set_mixer_gain, AOB_OP_SET_MIXER_GAIN)
SET_U8(airspy_set_vga_gain, AOB_OP_SET_VGA_GAIN)
SET_U8(airspy_set_lna_agc, AOB_OP_SET_LNA_AGC)
SET_U8(airspy_set_mixer_agc, AOB_OP_SET_MIXER_AGC)
SET_U8(airspy_set_linearity_gain, AOB_OP_SET_LINEARITY_GAIN)
SET_U8(airspy_set_sensitivity_gain, AOB_OP_SET_SENSITIVITY_GAIN)
SET_U8(airspy_set_rf_bias, AOB_OP_SET_RF_BIAS)
SET_U8(airspy_set_packing, AOB_OP_SET_PACKING)

int ADDCALL airspy_set_sample_type(struct airspy_device *device,
                                  enum airspy_sample_type sample_type)
{
    uint32_t value = (uint32_t)sample_type;
    return device_rpc(device, AOB_OP_SET_SAMPLE_TYPE, &value, sizeof(value), NULL, 0);
}

int ADDCALL airspy_set_conversion_filter_float32(struct airspy_device *device,
                                                 const float *kernel, uint32_t length)
{
    (void)device; (void)kernel; (void)length;
    return AIRSPY_ERROR_UNSUPPORTED;
}

int ADDCALL airspy_set_conversion_filter_int16(struct airspy_device *device,
                                               const int16_t *kernel, uint32_t length)
{
    (void)device; (void)kernel; (void)length;
    return AIRSPY_ERROR_UNSUPPORTED;
}

int ADDCALL airspy_board_id_read(struct airspy_device *device, uint8_t *value)
{
    return device_rpc(device, AOB_OP_BOARD_ID_READ, NULL, 0, value, sizeof(*value));
}

int ADDCALL airspy_version_string_read(struct airspy_device *device, char *version,
                                       uint8_t length)
{
    return device_rpc(device, AOB_OP_VERSION_STRING_READ, &length, sizeof(length),
                      version, length);
}

int ADDCALL airspy_board_partid_serialno_read(
    struct airspy_device *device, airspy_read_partid_serialno_t *value)
{
    return device_rpc(device, AOB_OP_BOARD_PARTID_SERIALNO_READ, NULL, 0,
                      value, sizeof(*value));
}

static int register_write(struct airspy_device *device, uint32_t op,
                          uint8_t reg, uint8_t value)
{
    uint8_t input[2] = { reg, value };
    return device_rpc(device, op, input, sizeof(input), NULL, 0);
}

static int register_read(struct airspy_device *device, uint32_t op,
                         uint8_t reg, uint8_t *value)
{
    return device_rpc(device, op, &reg, sizeof(reg), value, sizeof(*value));
}

int ADDCALL airspy_si5351c_write(struct airspy_device *d, uint8_t r, uint8_t v)
{ return register_write(d, AOB_OP_SI5351C_WRITE, r, v); }
int ADDCALL airspy_si5351c_read(struct airspy_device *d, uint8_t r, uint8_t *v)
{ return register_read(d, AOB_OP_SI5351C_READ, r, v); }
int ADDCALL airspy_r820t_write(struct airspy_device *d, uint8_t r, uint8_t v)
{ return register_write(d, AOB_OP_R820T_WRITE, r, v); }
int ADDCALL airspy_r820t_read(struct airspy_device *d, uint8_t r, uint8_t *v)
{ return register_read(d, AOB_OP_R820T_READ, r, v); }

static int gpio_write(struct airspy_device *device, uint32_t op,
                      int32_t port, int32_t pin, uint8_t value)
{
    int32_t input[3] = { port, pin, value };
    return device_rpc(device, op, input, sizeof(input), NULL, 0);
}

static int gpio_read(struct airspy_device *device, uint32_t op,
                     int32_t port, int32_t pin, uint8_t *value)
{
    int32_t input[2] = { port, pin };
    return device_rpc(device, op, input, sizeof(input), value, sizeof(*value));
}

int ADDCALL airspy_gpio_write(struct airspy_device *d, airspy_gpio_port_t p,
                              airspy_gpio_pin_t n, uint8_t v)
{ return gpio_write(d, AOB_OP_GPIO_WRITE, p, n, v); }
int ADDCALL airspy_gpio_read(struct airspy_device *d, airspy_gpio_port_t p,
                             airspy_gpio_pin_t n, uint8_t *v)
{ return gpio_read(d, AOB_OP_GPIO_READ, p, n, v); }
int ADDCALL airspy_gpiodir_write(struct airspy_device *d, airspy_gpio_port_t p,
                                 airspy_gpio_pin_t n, uint8_t v)
{ return gpio_write(d, AOB_OP_GPIODIR_WRITE, p, n, v); }
int ADDCALL airspy_gpiodir_read(struct airspy_device *d, airspy_gpio_port_t p,
                                airspy_gpio_pin_t n, uint8_t *v)
{ return gpio_read(d, AOB_OP_GPIODIR_READ, p, n, v); }

int ADDCALL airspy_spiflash_erase(struct airspy_device *device)
{ return device_rpc(device, AOB_OP_SPIFLASH_ERASE, NULL, 0, NULL, 0); }

int ADDCALL airspy_spiflash_erase_sector(struct airspy_device *device,
                                        uint16_t sector)
{ return device_rpc(device, AOB_OP_SPIFLASH_ERASE_SECTOR,
                    &sector, sizeof(sector), NULL, 0); }

int ADDCALL airspy_spiflash_write(struct airspy_device *device, uint32_t address,
                                  uint16_t length, unsigned char *const data)
{
    (void)device; (void)address; (void)length; (void)data;
    return AIRSPY_ERROR_UNSUPPORTED;
}

int ADDCALL airspy_spiflash_read(struct airspy_device *device, uint32_t address,
                                 uint16_t length, unsigned char *data)
{
    (void)device; (void)address; (void)length; (void)data;
    return AIRSPY_ERROR_UNSUPPORTED;
}

const char * ADDCALL airspy_error_name(enum airspy_error error)
{
    switch (error) {
    case AIRSPY_SUCCESS: return "AIRSPY_SUCCESS";
    case AIRSPY_TRUE: return "AIRSPY_TRUE";
    case AIRSPY_ERROR_INVALID_PARAM: return "AIRSPY_ERROR_INVALID_PARAM";
    case AIRSPY_ERROR_NOT_FOUND: return "AIRSPY_ERROR_NOT_FOUND";
    case AIRSPY_ERROR_BUSY: return "AIRSPY_ERROR_BUSY";
    case AIRSPY_ERROR_NO_MEM: return "AIRSPY_ERROR_NO_MEM";
    case AIRSPY_ERROR_UNSUPPORTED: return "AIRSPY_ERROR_UNSUPPORTED";
    case AIRSPY_ERROR_LIBUSB: return "AIRSPY_ERROR_LIBUSB";
    case AIRSPY_ERROR_THREAD: return "AIRSPY_ERROR_THREAD";
    case AIRSPY_ERROR_STREAMING_THREAD_ERR: return "AIRSPY_ERROR_STREAMING_THREAD_ERR";
    case AIRSPY_ERROR_STREAMING_STOPPED: return "AIRSPY_ERROR_STREAMING_STOPPED";
    default: return "AIRSPY_ERROR_OTHER";
    }
}

const char * ADDCALL airspy_board_id_name(enum airspy_board_id board)
{
    return board == AIRSPY_BOARD_ID_PROTO_AIRSPY ? "AIRSPY" : "INVALID";
}
