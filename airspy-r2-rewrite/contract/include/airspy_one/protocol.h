#ifndef AIRSPY_ONE_PROTOCOL_H
#define AIRSPY_ONE_PROTOCOL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

enum {
    AIRSPY_ONE_USB_VID = 0x1d50,
    AIRSPY_ONE_USB_PID = 0x60a1,
    AIRSPY_ONE_INTERFACE = 0,
    AIRSPY_ONE_ALT_SETTING = 0,
    AIRSPY_ONE_BULK_IN_ENDPOINT = 0x81,
    AIRSPY_ONE_BULK_OUT_ENDPOINT = 0x02,
    AIRSPY_ONE_EP0_PACKET_BYTES = 64,
    AIRSPY_ONE_HS_BULK_PACKET_BYTES = 512
};

typedef enum airspy_one_legacy_request {
    AIRSPY_ONE_INVALID = 0,
    AIRSPY_ONE_RECEIVER_MODE = 1,
    AIRSPY_ONE_SI5351C_WRITE = 2,
    AIRSPY_ONE_SI5351C_READ = 3,
    AIRSPY_ONE_R820T_WRITE = 4,
    AIRSPY_ONE_R820T_READ = 5,
    AIRSPY_ONE_SPIFLASH_ERASE = 6,
    AIRSPY_ONE_SPIFLASH_WRITE = 7,
    AIRSPY_ONE_SPIFLASH_READ = 8,
    AIRSPY_ONE_BOARD_ID_READ = 9,
    AIRSPY_ONE_VERSION_STRING_READ = 10,
    AIRSPY_ONE_BOARD_PARTID_SERIALNO_READ = 11,
    AIRSPY_ONE_SET_SAMPLERATE = 12,
    AIRSPY_ONE_SET_FREQ = 13,
    AIRSPY_ONE_SET_LNA_GAIN = 14,
    AIRSPY_ONE_SET_MIXER_GAIN = 15,
    AIRSPY_ONE_SET_VGA_GAIN = 16,
    AIRSPY_ONE_SET_LNA_AGC = 17,
    AIRSPY_ONE_SET_MIXER_AGC = 18,
    AIRSPY_ONE_MS_VENDOR_CMD = 19,
    AIRSPY_ONE_SET_RF_BIAS = 20,
    AIRSPY_ONE_GPIO_WRITE = 21,
    AIRSPY_ONE_GPIO_READ = 22,
    AIRSPY_ONE_GPIODIR_WRITE = 23,
    AIRSPY_ONE_GPIODIR_READ = 24,
    AIRSPY_ONE_GET_SAMPLERATES = 25,
    AIRSPY_ONE_SET_PACKING = 26,
    AIRSPY_ONE_SPIFLASH_ERASE_SECTOR = 27
} airspy_one_legacy_request;

enum {
    AIRSPY_ONE_EXPERIMENTAL_GET_PROTOCOL_INFO = 0x80,
    AIRSPY_ONE_EXPERIMENTAL_GET_CAPABILITIES = 0x81,
    AIRSPY_ONE_EXPERIMENTAL_SET_STREAM_CONFIG = 0x82,
    AIRSPY_ONE_EXPERIMENTAL_GET_STREAM_CONFIG = 0x83,
    AIRSPY_ONE_EXPERIMENTAL_GET_STREAM_STATUS = 0x84,
    AIRSPY_ONE_EXPERIMENTAL_CLEAR_STREAM_STATUS = 0x85,
    AIRSPY_ONE_PROTOCOL_INFO_WIRE_BYTES = 24,
    AIRSPY_ONE_STREAM_STATUS_WIRE_BYTES = 64
};

typedef enum airspy_one_stream_format {
    AIRSPY_ONE_FORMAT_RAW_U12_LE16 = 0,
    AIRSPY_ONE_FORMAT_PACKED_U12_LEGACY = 1,
    AIRSPY_ONE_FORMAT_LOSSLESS_BLOCK_V2 = 2
} airspy_one_stream_format;

typedef enum airspy_one_capability_id {
    AIRSPY_ONE_CAP_HARDWARE_MODEL = 1,
    AIRSPY_ONE_CAP_SAMPLE_RATES = 2,
    AIRSPY_ONE_CAP_STREAM_FORMATS = 3,
    AIRSPY_ONE_CAP_FRAME_LIMITS = 4,
    AIRSPY_ONE_CAP_SEQUENCE_REPORTING = 5,
    AIRSPY_ONE_CAP_DEVICE_LOSS_REPORTING = 6,
    AIRSPY_ONE_CAP_QUEUE_DEPTH = 7,
    AIRSPY_ONE_CAP_TEST_PATTERN = 8
} airspy_one_capability_id;

typedef struct airspy_one_protocol_info {
    uint8_t contract_major;
    uint8_t contract_minor;
    uint32_t schema_revision;
    uint32_t firmware_revision;
    uint16_t hardware_model;
    uint16_t hardware_revision;
    uint16_t capability_bytes;
    uint16_t maximum_control_payload;
} airspy_one_protocol_info;

typedef enum airspy_one_stream_status_flag {
    AIRSPY_ONE_STREAM_RUNNING = 1u << 0,
    AIRSPY_ONE_STREAM_DEVICE_LOSS = 1u << 1,
    AIRSPY_ONE_STREAM_USB_ERROR = 1u << 2,
    AIRSPY_ONE_STREAM_CAPTURE_BACKPRESSURE = 1u << 3
} airspy_one_stream_status_flag;

typedef struct airspy_one_stream_status {
    uint32_t generation;
    uint32_t flags;
    uint64_t captured_samples;
    uint64_t submitted_samples;
    uint64_t retired_samples;
    uint64_t dropped_samples;
    uint32_t backpressure_events;
    uint16_t queue_high_watermark;
    uint8_t capture_buffer_count;
    uint8_t descriptor_count;
    uint32_t completion_count;
} airspy_one_stream_status;

int airspy_one_protocol_info_encode(
    uint8_t destination[AIRSPY_ONE_PROTOCOL_INFO_WIRE_BYTES],
    const airspy_one_protocol_info* information);

int airspy_one_protocol_info_decode(
    airspy_one_protocol_info* information,
    const uint8_t* source,
    size_t source_length);

int airspy_one_stream_status_encode(
    uint8_t destination[AIRSPY_ONE_STREAM_STATUS_WIRE_BYTES],
    const airspy_one_stream_status* status);

int airspy_one_stream_status_decode(
    airspy_one_stream_status* status,
    const uint8_t* source,
    size_t source_length);

int airspy_one_capability_next(
    const uint8_t* source,
    size_t source_length,
    size_t* cursor,
    uint16_t* capability_id,
    const uint8_t** value,
    uint16_t* value_length);

#ifdef __cplusplus
}
#endif

#endif
