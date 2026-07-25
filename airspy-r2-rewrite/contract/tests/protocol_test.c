#include "airspy_one/protocol.h"

#include <assert.h>
#include <string.h>

int main(void)
{
    const airspy_one_protocol_info input = {
        .contract_major = 2,
        .contract_minor = 0,
        .schema_revision = 3,
        .firmware_revision = 0x01020304,
        .hardware_model = 2,
        .hardware_revision = 1,
        .capability_bytes = 12,
        .maximum_control_payload = 256
    };
    airspy_one_protocol_info output;
    uint8_t wire[AIRSPY_ONE_PROTOCOL_INFO_WIRE_BYTES];

    assert(airspy_one_protocol_info_encode(wire, &input) == 0);
    assert(memcmp(wire, "ASR2", 4) == 0);
    assert(airspy_one_protocol_info_decode(&output, wire, sizeof(wire)) == 0);
    assert(output.contract_major == input.contract_major);
    assert(output.schema_revision == input.schema_revision);
    assert(output.firmware_revision == input.firmware_revision);
    assert(output.hardware_model == input.hardware_model);
    assert(output.capability_bytes == input.capability_bytes);
    assert(output.maximum_control_payload == input.maximum_control_payload);

    wire[0] = 'X';
    assert(airspy_one_protocol_info_decode(&output, wire, sizeof(wire)) == -2);
    assert(airspy_one_protocol_info_decode(&output, wire, 12) == -1);

    {
        const airspy_one_stream_status status_input = {
            .generation = 14,
            .flags = AIRSPY_ONE_STREAM_RUNNING
                | AIRSPY_ONE_STREAM_CAPTURE_BACKPRESSURE,
            .captured_samples = UINT64_C(0x123456789abcdef0),
            .submitted_samples = UINT64_C(0x123456789abc0000),
            .retired_samples = UINT64_C(0x123456789abb0000),
            .dropped_samples = 8192,
            .backpressure_events = 7,
            .queue_high_watermark = 3,
            .capture_buffer_count = 4,
            .descriptor_count = 3,
            .completion_count = 65539
        };
        airspy_one_stream_status status_output;
        uint8_t status_wire[AIRSPY_ONE_STREAM_STATUS_WIRE_BYTES];
        assert(airspy_one_stream_status_encode(status_wire, &status_input) == 0);
        assert(memcmp(status_wire, "AST2", 4) == 0);
        assert(status_wire[4] == 64 && status_wire[5] == 0);
        assert(status_wire[6] == 1 && status_wire[7] == 0);
        assert(status_wire[16] == 0xf0 && status_wire[17] == 0xde);
        assert(status_wire[22] == 0x34 && status_wire[23] == 0x12);
        assert(status_wire[52] == 3 && status_wire[53] == 0);
        assert(status_wire[54] == 4 && status_wire[55] == 3);
        assert(airspy_one_stream_status_decode(
                   &status_output, status_wire, sizeof(status_wire))
            == 0);
        assert(status_output.generation == status_input.generation);
        assert(status_output.flags == status_input.flags);
        assert(status_output.captured_samples == status_input.captured_samples);
        assert(status_output.submitted_samples == status_input.submitted_samples);
        assert(status_output.retired_samples == status_input.retired_samples);
        assert(status_output.dropped_samples == status_input.dropped_samples);
        assert(status_output.backpressure_events
            == status_input.backpressure_events);
        assert(status_output.queue_high_watermark
            == status_input.queue_high_watermark);
        assert(status_output.capture_buffer_count
            == status_input.capture_buffer_count);
        assert(status_output.descriptor_count == status_input.descriptor_count);
        assert(status_output.completion_count == status_input.completion_count);
        status_wire[6] = 2;
        assert(airspy_one_stream_status_decode(
                   &status_output, status_wire, sizeof(status_wire))
            == -2);
        assert(airspy_one_stream_status_decode(
                   &status_output, status_wire, 48)
            == -1);
    }

    {
        const uint8_t capabilities[] = {
            1, 0, 2, 0, 9, 0,
            7, 0, 2, 0, 4, 0};
        size_t cursor = 0;
        uint16_t id = 0;
        uint16_t length = 0;
        const uint8_t* value = NULL;
        assert(airspy_one_capability_next(
                   capabilities, sizeof(capabilities), &cursor, &id, &value, &length)
            == 0);
        assert(id == 1 && length == 2 && value[0] == 9);
        assert(airspy_one_capability_next(
                   capabilities, sizeof(capabilities), &cursor, &id, &value, &length)
            == 0);
        assert(id == 7 && length == 2 && value[0] == 4);
        assert(airspy_one_capability_next(
                   capabilities, sizeof(capabilities), &cursor, &id, &value, &length)
            == 1);
    }
    return 0;
}
