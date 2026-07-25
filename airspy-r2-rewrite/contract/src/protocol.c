#include "airspy_one/protocol.h"

#include <string.h>

static void write_u16_le(uint8_t* destination, uint16_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
}

static void write_u32_le(uint8_t* destination, uint32_t value)
{
    destination[0] = (uint8_t)value;
    destination[1] = (uint8_t)(value >> 8);
    destination[2] = (uint8_t)(value >> 16);
    destination[3] = (uint8_t)(value >> 24);
}

static void write_u64_le(uint8_t* destination, uint64_t value)
{
    write_u32_le(destination, (uint32_t)value);
    write_u32_le(destination + 4, (uint32_t)(value >> 32));
}

static uint16_t read_u16_le(const uint8_t* source)
{
    return (uint16_t)((uint16_t)source[0] | ((uint16_t)source[1] << 8));
}

static uint32_t read_u32_le(const uint8_t* source)
{
    return (uint32_t)source[0] | ((uint32_t)source[1] << 8)
        | ((uint32_t)source[2] << 16) | ((uint32_t)source[3] << 24);
}

static uint64_t read_u64_le(const uint8_t* source)
{
    return (uint64_t)read_u32_le(source)
        | ((uint64_t)read_u32_le(source + 4) << 32);
}

int airspy_one_protocol_info_encode(
    uint8_t destination[AIRSPY_ONE_PROTOCOL_INFO_WIRE_BYTES],
    const airspy_one_protocol_info* information)
{
    if (destination == NULL || information == NULL) {
        return -1;
    }

    memset(destination, 0, AIRSPY_ONE_PROTOCOL_INFO_WIRE_BYTES);
    destination[0] = 'A';
    destination[1] = 'S';
    destination[2] = 'R';
    destination[3] = '2';
    write_u16_le(&destination[4], AIRSPY_ONE_PROTOCOL_INFO_WIRE_BYTES);
    destination[6] = information->contract_major;
    destination[7] = information->contract_minor;
    write_u32_le(&destination[8], information->schema_revision);
    write_u32_le(&destination[12], information->firmware_revision);
    write_u16_le(&destination[16], information->hardware_model);
    write_u16_le(&destination[18], information->hardware_revision);
    write_u16_le(&destination[20], information->capability_bytes);
    write_u16_le(&destination[22], information->maximum_control_payload);
    return 0;
}

int airspy_one_protocol_info_decode(
    airspy_one_protocol_info* information,
    const uint8_t* source,
    size_t source_length)
{
    if (information == NULL || source == NULL
        || source_length < AIRSPY_ONE_PROTOCOL_INFO_WIRE_BYTES) {
        return -1;
    }
    if (source[0] != 'A' || source[1] != 'S' || source[2] != 'R'
        || source[3] != '2'
        || read_u16_le(&source[4]) != AIRSPY_ONE_PROTOCOL_INFO_WIRE_BYTES) {
        return -2;
    }

    information->contract_major = source[6];
    information->contract_minor = source[7];
    information->schema_revision = read_u32_le(&source[8]);
    information->firmware_revision = read_u32_le(&source[12]);
    information->hardware_model = read_u16_le(&source[16]);
    information->hardware_revision = read_u16_le(&source[18]);
    information->capability_bytes = read_u16_le(&source[20]);
    information->maximum_control_payload = read_u16_le(&source[22]);
    return 0;
}

int airspy_one_stream_status_encode(
    uint8_t destination[AIRSPY_ONE_STREAM_STATUS_WIRE_BYTES],
    const airspy_one_stream_status* status)
{
    if (destination == NULL || status == NULL) {
        return -1;
    }

    memset(destination, 0, AIRSPY_ONE_STREAM_STATUS_WIRE_BYTES);
    destination[0] = 'A';
    destination[1] = 'S';
    destination[2] = 'T';
    destination[3] = '2';
    write_u16_le(&destination[4], AIRSPY_ONE_STREAM_STATUS_WIRE_BYTES);
    write_u16_le(&destination[6], 1);
    write_u32_le(&destination[8], status->generation);
    write_u32_le(&destination[12], status->flags);
    write_u64_le(&destination[16], status->captured_samples);
    write_u64_le(&destination[24], status->submitted_samples);
    write_u64_le(&destination[32], status->retired_samples);
    write_u64_le(&destination[40], status->dropped_samples);
    write_u32_le(&destination[48], status->backpressure_events);
    write_u16_le(&destination[52], status->queue_high_watermark);
    destination[54] = status->capture_buffer_count;
    destination[55] = status->descriptor_count;
    write_u32_le(&destination[56], status->completion_count);
    return 0;
}

int airspy_one_stream_status_decode(
    airspy_one_stream_status* status,
    const uint8_t* source,
    size_t source_length)
{
    if (status == NULL || source == NULL
        || source_length < AIRSPY_ONE_STREAM_STATUS_WIRE_BYTES) {
        return -1;
    }
    if (source[0] != 'A' || source[1] != 'S' || source[2] != 'T'
        || source[3] != '2'
        || read_u16_le(&source[4]) != AIRSPY_ONE_STREAM_STATUS_WIRE_BYTES
        || read_u16_le(&source[6]) != 1) {
        return -2;
    }

    status->generation = read_u32_le(&source[8]);
    status->flags = read_u32_le(&source[12]);
    status->captured_samples = read_u64_le(&source[16]);
    status->submitted_samples = read_u64_le(&source[24]);
    status->retired_samples = read_u64_le(&source[32]);
    status->dropped_samples = read_u64_le(&source[40]);
    status->backpressure_events = read_u32_le(&source[48]);
    status->queue_high_watermark = read_u16_le(&source[52]);
    status->capture_buffer_count = source[54];
    status->descriptor_count = source[55];
    status->completion_count = read_u32_le(&source[56]);
    return 0;
}

int airspy_one_capability_next(
    const uint8_t* source,
    size_t source_length,
    size_t* cursor,
    uint16_t* capability_id,
    const uint8_t** value,
    uint16_t* value_length)
{
    if (source == NULL || cursor == NULL || capability_id == NULL
        || value == NULL || value_length == NULL || *cursor > source_length) {
        return -1;
    }
    if (*cursor == source_length) {
        return 1;
    }
    if (source_length - *cursor < 4) {
        return -2;
    }

    const uint16_t id = read_u16_le(&source[*cursor]);
    const uint16_t length = read_u16_le(&source[*cursor + 2]);
    if ((size_t)length > source_length - *cursor - 4) {
        return -2;
    }
    *capability_id = id;
    *value_length = length;
    *value = &source[*cursor + 4];
    *cursor += 4 + length;
    return 0;
}
