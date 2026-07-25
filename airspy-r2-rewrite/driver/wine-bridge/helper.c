/*
 * Native macOS side of the Airspy One Wine bridge.
 *
 * This process owns libusb and the radio.  The Wine process only sees the
 * stable public airspy.dll ABI implemented by shim.c.
 */
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "airspy.h"
#include "libusb.h"
#include "proto.h"

#define MAX_DEVICES 8
#define LOG(...) do { fprintf(stderr, "[airspy-helper] " __VA_ARGS__); fputc('\n', stderr); } while (0)

typedef struct {
    int in_use;
    uint64_t id;
    struct airspy_device *device;
    int data_fd;
    uint8_t packing;
    pthread_mutex_t control_lock;
    pthread_mutex_t data_lock;
} device_record;

static device_record records[MAX_DEVICES];
static pthread_mutex_t registry_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t monitor_lock = PTHREAD_MUTEX_INITIALIZER;
static uint64_t next_id = 1;
static int verbose;
static uint64_t monitor_epoch = 1;
static uint32_t monitor_event = AOB_MONITOR_HELPER_STARTED;
static uint32_t monitor_streaming;
static uint32_t monitor_sample_rate;
static uint64_t monitor_host_dropped_samples;

/*
 * The bridge is pinned to the adjacent libairspy build. These are the first
 * two fields of that build's private airspy_device_t and give the helper a
 * read-only path for the firmware's diagnostic vendor request without asking
 * a second macOS process to compete for the USB device.
 */
typedef struct {
    libusb_context *usb_context;
    libusb_device_handle *usb_device;
} airspy_device_usb_prefix;

static device_record *find_record(uint64_t id);

static void monitor_publish(uint32_t event, int streaming,
                            uint32_t sample_rate, int replace_rate)
{
    pthread_mutex_lock(&monitor_lock);
    monitor_epoch++;
    monitor_event = event;
    if (streaming >= 0) monitor_streaming = streaming != 0;
    if (replace_rate) monitor_sample_rate = sample_rate;
    pthread_mutex_unlock(&monitor_lock);
}

static device_record *lock_record_by_id(uint64_t id)
{
    device_record *candidate = find_record(id);
    if (!candidate) return NULL;

    pthread_mutex_lock(&candidate->control_lock);
    pthread_mutex_lock(&registry_lock);
    int valid = candidate->in_use && candidate->id == id &&
                candidate->device != NULL;
    pthread_mutex_unlock(&registry_lock);
    if (!valid) {
        pthread_mutex_unlock(&candidate->control_lock);
        return NULL;
    }
    return candidate;
}

static device_record *lock_first_record(void)
{
    device_record *candidate = NULL;
    pthread_mutex_lock(&registry_lock);
    for (int i = 0; i < MAX_DEVICES; ++i) {
        if (records[i].in_use && records[i].device) {
            candidate = &records[i];
            break;
        }
    }
    pthread_mutex_unlock(&registry_lock);
    if (!candidate) return NULL;

    pthread_mutex_lock(&candidate->control_lock);
    pthread_mutex_lock(&registry_lock);
    int valid = candidate->in_use && candidate->device != NULL;
    pthread_mutex_unlock(&registry_lock);
    if (!valid) {
        pthread_mutex_unlock(&candidate->control_lock);
        return NULL;
    }
    return candidate;
}

static void fill_monitor_snapshot(aob_monitor_snapshot *snapshot)
{
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->magic = AOB_MONITOR_MAGIC;
    snapshot->version = AOB_MONITOR_VERSION;

    pthread_mutex_lock(&monitor_lock);
    snapshot->session_epoch = monitor_epoch;
    snapshot->event = monitor_event;
    snapshot->streaming = monitor_streaming;
    snapshot->sample_rate = monitor_sample_rate;
    snapshot->host_dropped_samples = monitor_host_dropped_samples;
    pthread_mutex_unlock(&monitor_lock);

    device_record *record = lock_first_record();
    if (!record) {
        snapshot->telemetry_result = AIRSPY_ERROR_NOT_FOUND;
        return;
    }

    airspy_device_usb_prefix *prefix =
        (airspy_device_usb_prefix *)record->device;
    int result = libusb_control_transfer(
        prefix->usb_device,
        LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR |
            LIBUSB_RECIPIENT_DEVICE,
        0x87, 0, 0, (unsigned char *)&snapshot->telemetry,
        sizeof(snapshot->telemetry), 500);

    snapshot->telemetry_result = result;
    snapshot->telemetry_valid =
        result == (int)sizeof(snapshot->telemetry) &&
        snapshot->telemetry.version == AOB_STREAM_CONTRACT_VERSION;

    /*
     * Stock firmware stalls 0x87. If it does, try the instrumented-vanilla
     * counter instead, so a vanilla comparison run can be watched live in the
     * same window. Stock firmware without the patch stalls this too, which
     * simply leaves vanilla_valid clear.
     */
    if (!snapshot->telemetry_valid) {
        unsigned char raw[4];
        int vanilla = libusb_control_transfer(
            prefix->usb_device,
            LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR |
                LIBUSB_RECIPIENT_DEVICE,
            28, 0, 0, raw, sizeof(raw), 500);
        if (vanilla == (int)sizeof(raw)) {
            snapshot->vanilla_valid = 1;
            snapshot->vanilla_fifo_overflow =
                (uint32_t)raw[0] | ((uint32_t)raw[1] << 8) |
                ((uint32_t)raw[2] << 16) | ((uint32_t)raw[3] << 24);
        }
    }

    pthread_mutex_unlock(&record->control_lock);
}

static int read_full(int fd, void *data, size_t length)
{
    uint8_t *cursor = (uint8_t *)data;
    while (length) {
        ssize_t received = recv(fd, cursor, length, 0);
        if (received == 0) return -1;
        if (received < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        cursor += received;
        length -= (size_t)received;
    }
    return 0;
}

static int write_full(int fd, const void *data, size_t length)
{
    const uint8_t *cursor = (const uint8_t *)data;
    while (length) {
        ssize_t sent = send(fd, cursor, length, 0);
        if (sent < 0) {
            if (errno == EINTR) continue;
            return -1;
        }
        cursor += sent;
        length -= (size_t)sent;
    }
    return 0;
}

static device_record *allocate_record(struct airspy_device *device)
{
    pthread_mutex_lock(&registry_lock);
    device_record *record = NULL;
    for (int i = 0; i < MAX_DEVICES; ++i) {
        if (!records[i].in_use) {
            record = &records[i];
            record->in_use = 1;
            record->id = next_id++;
            record->device = device;
            record->data_fd = -1;
            record->packing = 0;
            break;
        }
    }
    pthread_mutex_unlock(&registry_lock);
    return record;
}

static device_record *find_record(uint64_t id)
{
    pthread_mutex_lock(&registry_lock);
    device_record *record = NULL;
    for (int i = 0; i < MAX_DEVICES; ++i) {
        if (records[i].in_use && records[i].id == id) {
            record = &records[i];
            break;
        }
    }
    pthread_mutex_unlock(&registry_lock);
    return record;
}

static void release_record(device_record *record)
{
    pthread_mutex_lock(&record->data_lock);
    if (record->data_fd >= 0) {
        shutdown(record->data_fd, SHUT_RDWR);
        close(record->data_fd);
        record->data_fd = -1;
    }
    pthread_mutex_unlock(&record->data_lock);

    pthread_mutex_lock(&registry_lock);
    record->in_use = 0;
    record->device = NULL;
    pthread_mutex_unlock(&registry_lock);
}

static uint32_t sample_payload_bytes(const airspy_transfer_t *transfer,
                                     const device_record *record)
{
    uint64_t count = (uint32_t)transfer->sample_count;
    uint64_t bytes;
    switch (transfer->sample_type) {
    case AIRSPY_SAMPLE_FLOAT32_IQ: bytes = count * 2u * sizeof(float); break;
    case AIRSPY_SAMPLE_FLOAT32_REAL: bytes = count * sizeof(float); break;
    case AIRSPY_SAMPLE_INT16_IQ: bytes = count * 2u * sizeof(int16_t); break;
    case AIRSPY_SAMPLE_INT16_REAL:
    case AIRSPY_SAMPLE_UINT16_REAL: bytes = count * sizeof(uint16_t); break;
    case AIRSPY_SAMPLE_RAW:
        bytes = record->packing ? (count * 3u) / 2u : count * sizeof(uint16_t);
        break;
    default: return 0;
    }
    return bytes <= UINT32_MAX ? (uint32_t)bytes : 0;
}

static int sample_callback(airspy_transfer_t *transfer)
{
    device_record *record = (device_record *)transfer->ctx;
    uint32_t bytes = sample_payload_bytes(transfer, record);
    if (!bytes) return -1;
    pthread_mutex_lock(&monitor_lock);
    monitor_host_dropped_samples = transfer->dropped_samples;
    pthread_mutex_unlock(&monitor_lock);

    pthread_mutex_lock(&record->data_lock);
    int result = 0;
    if (record->data_fd >= 0) {
        aob_data_hdr header;
        header.magic = AOB_DATA_FRAME_MAGIC;
        header.sample_count = (uint32_t)transfer->sample_count;
        header.dropped_samples = transfer->dropped_samples;
        header.sample_type = (uint32_t)transfer->sample_type;
        header.payload_bytes = bytes;
        if (write_full(record->data_fd, &header, sizeof(header)) != 0 ||
            write_full(record->data_fd, transfer->samples, bytes) != 0) {
            shutdown(record->data_fd, SHUT_RDWR);
            close(record->data_fd);
            record->data_fd = -1;
            result = -1;
        }
    }
    pthread_mutex_unlock(&record->data_lock);
    return result;
}

static int reply(int fd, int32_t result, const void *output, uint32_t length)
{
    aob_resp_hdr response = { result, length };
    if (verbose) LOG("<- result=%d bytes=%u", result, length);
    if (write_full(fd, &response, sizeof(response)) != 0) return -1;
    if (length && write_full(fd, output, length) != 0) return -1;
    return 0;
}

static int one_u8(device_record *record, uint32_t op, uint8_t value)
{
    struct airspy_device *d = record ? record->device : NULL;
    if (!d) return AIRSPY_ERROR_INVALID_PARAM;
    switch (op) {
    case AOB_OP_SET_LNA_GAIN: return airspy_set_lna_gain(d, value);
    case AOB_OP_SET_MIXER_GAIN: return airspy_set_mixer_gain(d, value);
    case AOB_OP_SET_VGA_GAIN: return airspy_set_vga_gain(d, value);
    case AOB_OP_SET_LNA_AGC: return airspy_set_lna_agc(d, value);
    case AOB_OP_SET_MIXER_AGC: return airspy_set_mixer_agc(d, value);
    case AOB_OP_SET_LINEARITY_GAIN: return airspy_set_linearity_gain(d, value);
    case AOB_OP_SET_SENSITIVITY_GAIN: return airspy_set_sensitivity_gain(d, value);
    case AOB_OP_SET_RF_BIAS: return airspy_set_rf_bias(d, value);
    case AOB_OP_SET_PACKING:
        {
            int result = airspy_set_packing(d, value);
            if (result == AIRSPY_SUCCESS) record->packing = value != 0;
            return result;
        }
    default: return AIRSPY_ERROR_UNSUPPORTED;
    }
}

/*
 * Resolve an advertised rate back to its list index before calling the
 * upstream setter.  This matters for real-sample clients: get_samplerates()
 * doubles the advertised IQ rates, and on the Mini the resulting 6 MHz entry
 * collides numerically with the first 6 MHz IQ rate.  Passing the value back
 * directly therefore selects index 0 (12 MHz real) instead of index 1
 * (6 MHz real).  SDR# uses the real-sample API and exposes the collision as
 * exactly half-speed DSP/audio at its "3 MSPS Complex" setting.
 */
static int set_samplerate_resolved(device_record *record, uint32_t value)
{
    if (!record || !record->device) return AIRSPY_ERROR_INVALID_PARAM;
    if (value < 1000000u) return airspy_set_samplerate(record->device, value);

    uint32_t count = 0;
    int result = airspy_get_samplerates(record->device, &count, 0);
    if (result != AIRSPY_SUCCESS || !count) {
        return airspy_set_samplerate(record->device, value);
    }

    uint32_t *rates = (uint32_t *)calloc(count, sizeof(*rates));
    if (!rates) return AIRSPY_ERROR_NO_MEM;
    result = airspy_get_samplerates(record->device, rates, count);
    if (result == AIRSPY_SUCCESS) {
        for (uint32_t i = 0; i < count; ++i) {
            if (rates[i] == value) {
                value = i;
                break;
            }
        }
    }
    free(rates);
    return airspy_set_samplerate(record->device, value);
}

static void serve_control(int fd)
{
    device_record *owned = NULL;

    for (;;) {
        aob_req_hdr request;
        if (read_full(fd, &request, sizeof(request)) != 0) break;
        if (request.in_len > 1024u * 1024u) break;

        uint8_t *input = NULL;
        if (request.in_len) {
            input = (uint8_t *)malloc(request.in_len);
            if (!input || read_full(fd, input, request.in_len) != 0) {
                free(input);
                break;
            }
        }

        if (verbose) LOG("-> op=%u dev=%llu bytes=%u", request.op,
                         (unsigned long long)request.dev, request.in_len);
        device_record *record =
            request.dev && request.op != AOB_OP_GET_MONITOR_SNAPSHOT
                ? lock_record_by_id(request.dev) : NULL;
        int record_locked = record != NULL;
        struct airspy_device *device = record ? record->device : NULL;
        int32_t result = AIRSPY_ERROR_INVALID_PARAM;

        switch (request.op) {
        case AOB_OP_LIB_VERSION: {
            airspy_lib_version_t version = {0};
            airspy_lib_version(&version);
            reply(fd, AIRSPY_SUCCESS, &version, sizeof(version));
            break;
        }
        case AOB_OP_INIT:
            reply(fd, airspy_init(), NULL, 0);
            break;
        case AOB_OP_EXIT:
            reply(fd, airspy_exit(), NULL, 0);
            break;
        case AOB_OP_LIST_DEVICES: {
            int count = 0;
            if (input && request.in_len >= sizeof(count)) memcpy(&count, input, sizeof(count));
            if (count < 0 || count > 1024) {
                reply(fd, AIRSPY_ERROR_INVALID_PARAM, NULL, 0);
            } else if (count == 0) {
                result = airspy_list_devices(NULL, 0);
                reply(fd, result, NULL, 0);
            } else {
                uint64_t *serials = (uint64_t *)calloc((size_t)count, sizeof(*serials));
                result = airspy_list_devices(serials, count);
                uint32_t returned = result > 0 && result < count ? (uint32_t)result : (uint32_t)count;
                reply(fd, result, serials, returned * sizeof(*serials));
                free(serials);
            }
            break;
        }
        case AOB_OP_OPEN:
        case AOB_OP_OPEN_SN: {
            struct airspy_device *opened = NULL;
            if (request.op == AOB_OP_OPEN) {
                result = airspy_open(&opened);
            } else if (input && request.in_len >= sizeof(uint64_t)) {
                uint64_t serial;
                memcpy(&serial, input, sizeof(serial));
                result = airspy_open_sn(&opened, serial);
            }
            if (result == AIRSPY_SUCCESS && opened) {
                owned = allocate_record(opened);
                if (!owned) {
                    airspy_close(opened);
                    reply(fd, AIRSPY_ERROR_BUSY, NULL, 0);
                } else {
                    monitor_publish(
                        AOB_MONITOR_DEVICE_OPENED, 0, 0, 1);
                    reply(fd, AIRSPY_SUCCESS, &owned->id, sizeof(owned->id));
                }
            } else {
                reply(fd, result, NULL, 0);
            }
            break;
        }
        case AOB_OP_CLOSE:
            if (device) {
                if (airspy_is_streaming(device) == AIRSPY_TRUE) airspy_stop_rx(device);
                result = airspy_close(device);
            }
            reply(fd, result, NULL, 0);
            if (record) {
                if (owned == record) owned = NULL;
                release_record(record);
                monitor_publish(
                    AOB_MONITOR_DEVICE_CLOSED, 0, 0, 0);
            }
            break;
        case AOB_OP_GET_SAMPLERATES: {
            uint32_t length = 0;
            if (input && request.in_len >= sizeof(length)) memcpy(&length, input, sizeof(length));
            uint32_t count = length ? length : 1;
            uint32_t *rates = (uint32_t *)calloc(count, sizeof(*rates));
            result = device ? airspy_get_samplerates(device, rates, length)
                            : AIRSPY_ERROR_INVALID_PARAM;
            reply(fd, result, rates, count * sizeof(*rates));
            free(rates);
            break;
        }
        case AOB_OP_SET_SAMPLERATE:
        case AOB_OP_SET_FREQ:
        case AOB_OP_SET_SAMPLE_TYPE: {
            uint32_t value = 0;
            if (input && request.in_len >= sizeof(value)) memcpy(&value, input, sizeof(value));
            if (verbose) LOG("   value=%u", value);
            if (!device) result = AIRSPY_ERROR_INVALID_PARAM;
            else if (request.op == AOB_OP_SET_SAMPLERATE)
                result = set_samplerate_resolved(record, value);
            else if (request.op == AOB_OP_SET_FREQ)
                result = airspy_set_freq(device, value);
            else
                result = airspy_set_sample_type(device, (enum airspy_sample_type)value);
            if (request.op == AOB_OP_SET_SAMPLERATE &&
                result == AIRSPY_SUCCESS) {
                monitor_publish(AOB_MONITOR_SAMPLERATE_CHANGED,
                                -1, value, 1);
            }
            reply(fd, result, NULL, 0);
            break;
        }
        case AOB_OP_START_RX:
            result = device ? airspy_start_rx(device, sample_callback, record)
                            : AIRSPY_ERROR_INVALID_PARAM;
            if (result == AIRSPY_SUCCESS)
                monitor_publish(AOB_MONITOR_STREAM_STARTED, 1, 0, 0);
            reply(fd, result, NULL, 0);
            break;
        case AOB_OP_STOP_RX:
            result = device ? airspy_stop_rx(device) : AIRSPY_ERROR_INVALID_PARAM;
            if (result == AIRSPY_SUCCESS)
                monitor_publish(AOB_MONITOR_STREAM_STOPPED, 0, 0, 0);
            reply(fd, result, NULL, 0);
            break;
        case AOB_OP_IS_STREAMING:
            result = device ? airspy_is_streaming(device) : AIRSPY_ERROR_INVALID_PARAM;
            reply(fd, result, NULL, 0);
            break;
        case AOB_OP_SET_LNA_GAIN:
        case AOB_OP_SET_MIXER_GAIN:
        case AOB_OP_SET_VGA_GAIN:
        case AOB_OP_SET_LNA_AGC:
        case AOB_OP_SET_MIXER_AGC:
        case AOB_OP_SET_LINEARITY_GAIN:
        case AOB_OP_SET_SENSITIVITY_GAIN:
        case AOB_OP_SET_RF_BIAS:
        case AOB_OP_SET_PACKING:
            result = input && request.in_len ? one_u8(record, request.op, input[0])
                                             : AIRSPY_ERROR_INVALID_PARAM;
            reply(fd, result, NULL, 0);
            break;
        case AOB_OP_BOARD_ID_READ: {
            uint8_t value = 0;
            result = device ? airspy_board_id_read(device, &value)
                            : AIRSPY_ERROR_INVALID_PARAM;
            reply(fd, result, &value, sizeof(value));
            break;
        }
        case AOB_OP_VERSION_STRING_READ: {
            uint8_t length = input && request.in_len ? input[0] : 0;
            char text[256] = {0};
            result = device ? airspy_version_string_read(device, text, length)
                            : AIRSPY_ERROR_INVALID_PARAM;
            reply(fd, result, text, length);
            break;
        }
        case AOB_OP_BOARD_PARTID_SERIALNO_READ: {
            airspy_read_partid_serialno_t identity;
            memset(&identity, 0, sizeof(identity));
            result = device ? airspy_board_partid_serialno_read(device, &identity)
                            : AIRSPY_ERROR_INVALID_PARAM;
            reply(fd, result, &identity, sizeof(identity));
            break;
        }
        case AOB_OP_SI5351C_WRITE:
        case AOB_OP_R820T_WRITE: {
            if (!device || !input || request.in_len < 2) result = AIRSPY_ERROR_INVALID_PARAM;
            else if (request.op == AOB_OP_SI5351C_WRITE)
                result = airspy_si5351c_write(device, input[0], input[1]);
            else
                result = airspy_r820t_write(device, input[0], input[1]);
            reply(fd, result, NULL, 0);
            break;
        }
        case AOB_OP_SI5351C_READ:
        case AOB_OP_R820T_READ: {
            uint8_t value = 0;
            if (!device || !input || request.in_len < 1) result = AIRSPY_ERROR_INVALID_PARAM;
            else if (request.op == AOB_OP_SI5351C_READ)
                result = airspy_si5351c_read(device, input[0], &value);
            else
                result = airspy_r820t_read(device, input[0], &value);
            reply(fd, result, &value, sizeof(value));
            break;
        }
        case AOB_OP_GPIO_WRITE:
        case AOB_OP_GPIODIR_WRITE: {
            int32_t args[3] = {0};
            if (!device || !input || request.in_len < sizeof(args)) {
                result = AIRSPY_ERROR_INVALID_PARAM;
            } else {
                memcpy(args, input, sizeof(args));
                if (request.op == AOB_OP_GPIO_WRITE)
                    result = airspy_gpio_write(device, (airspy_gpio_port_t)args[0],
                                               (airspy_gpio_pin_t)args[1], (uint8_t)args[2]);
                else
                    result = airspy_gpiodir_write(device, (airspy_gpio_port_t)args[0],
                                                  (airspy_gpio_pin_t)args[1], (uint8_t)args[2]);
            }
            reply(fd, result, NULL, 0);
            break;
        }
        case AOB_OP_GPIO_READ:
        case AOB_OP_GPIODIR_READ: {
            int32_t args[2] = {0};
            uint8_t value = 0;
            if (!device || !input || request.in_len < sizeof(args)) {
                result = AIRSPY_ERROR_INVALID_PARAM;
            } else {
                memcpy(args, input, sizeof(args));
                if (request.op == AOB_OP_GPIO_READ)
                    result = airspy_gpio_read(device, (airspy_gpio_port_t)args[0],
                                              (airspy_gpio_pin_t)args[1], &value);
                else
                    result = airspy_gpiodir_read(device, (airspy_gpio_port_t)args[0],
                                                 (airspy_gpio_pin_t)args[1], &value);
            }
            reply(fd, result, &value, sizeof(value));
            break;
        }
        case AOB_OP_SPIFLASH_ERASE:
            result = device ? airspy_spiflash_erase(device) : AIRSPY_ERROR_INVALID_PARAM;
            reply(fd, result, NULL, 0);
            break;
        case AOB_OP_SPIFLASH_ERASE_SECTOR: {
            uint16_t sector = 0;
            if (input && request.in_len >= sizeof(sector)) memcpy(&sector, input, sizeof(sector));
            result = device ? airspy_spiflash_erase_sector(device, sector)
                            : AIRSPY_ERROR_INVALID_PARAM;
            reply(fd, result, NULL, 0);
            break;
        }
        case AOB_OP_GET_MONITOR_SNAPSHOT: {
            aob_monitor_snapshot snapshot;
            fill_monitor_snapshot(&snapshot);
            reply(fd, AIRSPY_SUCCESS, &snapshot, sizeof(snapshot));
            break;
        }
        default:
            reply(fd, AIRSPY_ERROR_UNSUPPORTED, NULL, 0);
            break;
        }
        if (record_locked)
            pthread_mutex_unlock(&record->control_lock);
        free(input);
    }

    if (owned && owned->in_use) {
        pthread_mutex_lock(&owned->control_lock);
        if (owned->device) {
            if (airspy_is_streaming(owned->device) == AIRSPY_TRUE)
                airspy_stop_rx(owned->device);
            airspy_close(owned->device);
        }
        release_record(owned);
        pthread_mutex_unlock(&owned->control_lock);
        monitor_publish(AOB_MONITOR_DEVICE_CLOSED, 0, 0, 0);
    }
    close(fd);
}

static void serve_data(int fd)
{
    aob_data_hello hello;
    if (read_full(fd, &hello, sizeof(hello)) != 0) {
        close(fd);
        return;
    }
    device_record *record = find_record(hello.dev);
    if (!record) {
        close(fd);
        return;
    }

    int one = 1;
    int buffer_size = 4 * 1024 * 1024;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size));
    pthread_mutex_lock(&record->data_lock);
    if (record->data_fd >= 0) close(record->data_fd);
    record->data_fd = fd;
    pthread_mutex_unlock(&record->data_lock);
    LOG("data channel bound to device %llu", (unsigned long long)hello.dev);

    uint8_t scratch[16];
    while (recv(fd, scratch, sizeof(scratch), 0) > 0) {}

    int still_owned = 0;
    pthread_mutex_lock(&record->data_lock);
    if (record->data_fd == fd) {
        record->data_fd = -1;
        still_owned = 1;
    }
    pthread_mutex_unlock(&record->data_lock);
    if (still_owned) close(fd);
}

typedef struct { int fd; } connection_argument;

static void *connection_thread(void *argument)
{
    int fd = ((connection_argument *)argument)->fd;
    free(argument);
    uint32_t channel = 0;
    if (read_full(fd, &channel, sizeof(channel)) != 0) {
        close(fd);
    } else if (channel == AOB_CHANNEL_CONTROL) {
        serve_control(fd);
    } else if (channel == AOB_CHANNEL_DATA) {
        serve_data(fd);
    } else {
        close(fd);
    }
    return NULL;
}

int main(void)
{
    signal(SIGPIPE, SIG_IGN);
    verbose = getenv("AIRSPY_BRIDGE_VERBOSE") != NULL;
    for (int i = 0; i < MAX_DEVICES; ++i) {
        records[i].data_fd = -1;
        pthread_mutex_init(&records[i].control_lock, NULL);
        pthread_mutex_init(&records[i].data_lock, NULL);
    }

    int port = AOB_DEFAULT_PORT;
    const char *configured = getenv(AOB_PORT_ENV);
    if (configured && atoi(configured) > 0) port = atoi(configured);

    int server = socket(AF_INET, SOCK_STREAM, 0);
    if (server < 0) {
        perror("socket");
        return 1;
    }
    int one = 1;
    setsockopt(server, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in address;
    memset(&address, 0, sizeof(address));
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    address.sin_port = htons((uint16_t)port);
    if (bind(server, (struct sockaddr *)&address, sizeof(address)) != 0 ||
        listen(server, 16) != 0) {
        perror("listen");
        close(server);
        return 1;
    }

    airspy_lib_version_t version = {0};
    airspy_lib_version(&version);
    LOG("listening on 127.0.0.1:%d (libairspy %u.%u.%u)", port,
        version.major_version, version.minor_version, version.revision);

    for (;;) {
        int fd = accept(server, NULL, NULL);
        if (fd < 0) {
            if (errno == EINTR) continue;
            break;
        }
        connection_argument *argument = (connection_argument *)malloc(sizeof(*argument));
        if (!argument) {
            close(fd);
            continue;
        }
        argument->fd = fd;
        pthread_t thread;
        if (pthread_create(&thread, NULL, connection_thread, argument) != 0) {
            close(fd);
            free(argument);
            continue;
        }
        pthread_detach(thread);
    }
    close(server);
    return 0;
}
