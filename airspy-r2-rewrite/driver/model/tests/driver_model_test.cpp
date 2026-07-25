#include "airspy_driver/legacy_unpack.hpp"
#include "airspy_driver/stream_accounting.hpp"
#include "airspy_driver/stream_lifecycle.hpp"

#include <array>
#include <cassert>

int main()
{
    using namespace airspy::driver;

    {
        constexpr std::array<std::uint8_t, 12> encoded{
            0x00, 0x01, 0x00, 0x00, 0x40, 0x00,
            0x03, 0x20, 0x07, 0x60, 0x00, 0x05};
        constexpr std::array<std::uint16_t, 8> expected{0, 1, 2, 3, 4, 5, 6, 7};
        std::array<std::uint16_t, 8> decoded{};
        assert(unpack_legacy_u12(encoded, decoded) == UnpackResult::ok);
        assert(decoded == expected);
    }
    {
        constexpr std::array<std::uint8_t, 12> encoded{
            0x12, 0xff, 0x0f, 0x00, 0x9a, 0x78,
            0x56, 0x34, 0x00, 0xf8, 0xde, 0xbc};
        constexpr std::array<std::uint16_t, 8> expected{
            0x000, 0xfff, 0x123, 0x456, 0x789, 0xabc, 0xdef, 0x800};
        std::array<std::uint16_t, 8> decoded{};
        assert(unpack_legacy_u12(encoded, decoded) == UnpackResult::ok);
        assert(decoded == expected);
    }

    StreamLifecycle lifecycle;
    assert(lifecycle.state() == StreamState::closed);
    assert(lifecycle.open() == TransitionResult::ok);
    assert(lifecycle.begin_start() == TransitionResult::ok);
    assert(lifecycle.generation() == 1);
    assert(lifecycle.complete_start() == TransitionResult::ok);
    assert(lifecycle.begin_stop() == TransitionResult::ok);
    assert(lifecycle.complete_stop() == TransitionResult::ok);
    assert(lifecycle.close() == TransitionResult::ok);
    assert(lifecycle.complete_start() == TransitionResult::illegal);
    assert(lifecycle.illegal_transitions() == 1);

    StreamAccounting accounting;
    accounting.begin_generation(3);
    airspy_one_stream_status status{
        .generation = 3,
        .flags = AIRSPY_ONE_STREAM_RUNNING,
        .captured_samples = 32768,
        .submitted_samples = 24576,
        .retired_samples = 16384,
        .dropped_samples = 0,
        .backpressure_events = 1,
        .queue_high_watermark = 3,
        .capture_buffer_count = 4,
        .descriptor_count = 3,
        .completion_count = 2};
    assert(accounting.observe_device(status) == StatusObservation::ok);
    accounting.record_host_delivery(16384);
    assert(accounting.host_delivered_samples() == 16384);
    assert(!accounting.discontinuity_observed());

    status.retired_samples = 24576;
    status.completion_count = 3;
    status.dropped_samples = 8192;
    status.flags |= AIRSPY_ONE_STREAM_DEVICE_LOSS;
    assert(accounting.observe_device(status) == StatusObservation::ok);
    assert(accounting.discontinuity_observed());

    airspy_one_stream_status stale = status;
    stale.generation = 2;
    assert(accounting.observe_device(stale)
        == StatusObservation::stale_generation);
    airspy_one_stream_status regressed = status;
    regressed.retired_samples = 8192;
    assert(accounting.observe_device(regressed) == StatusObservation::regressed);
    airspy_one_stream_status inconsistent = status;
    inconsistent.retired_samples = inconsistent.submitted_samples + 1;
    assert(accounting.observe_device(inconsistent)
        == StatusObservation::inconsistent);
    return 0;
}
