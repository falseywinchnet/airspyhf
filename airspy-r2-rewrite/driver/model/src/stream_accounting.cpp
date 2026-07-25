#include "airspy_driver/stream_accounting.hpp"

namespace airspy::driver {

void StreamAccounting::begin_generation(const std::uint32_t generation) noexcept
{
    generation_ = generation;
    device_ = {};
    device_.generation = generation;
    host_delivered_samples_ = 0;
    host_dropped_samples_ = 0;
}

StatusObservation StreamAccounting::observe_device(
    const airspy_one_stream_status& status) noexcept
{
    if (status.generation != generation_) {
        return StatusObservation::stale_generation;
    }
    if (status.retired_samples > status.submitted_samples
        || status.submitted_samples > status.captured_samples
        || status.queue_high_watermark > status.descriptor_count) {
        return StatusObservation::inconsistent;
    }
    if (status.captured_samples < device_.captured_samples
        || status.submitted_samples < device_.submitted_samples
        || status.retired_samples < device_.retired_samples
        || status.dropped_samples < device_.dropped_samples
        || status.backpressure_events < device_.backpressure_events
        || status.completion_count < device_.completion_count) {
        return StatusObservation::regressed;
    }
    device_ = status;
    return StatusObservation::ok;
}

void StreamAccounting::record_host_delivery(const std::uint64_t samples) noexcept
{
    host_delivered_samples_ += samples;
}

void StreamAccounting::record_host_discontinuity(
    const std::uint64_t samples) noexcept
{
    host_dropped_samples_ += samples;
}

std::uint32_t StreamAccounting::generation() const noexcept
{
    return generation_;
}

const airspy_one_stream_status& StreamAccounting::device() const noexcept
{
    return device_;
}

std::uint64_t StreamAccounting::host_delivered_samples() const noexcept
{
    return host_delivered_samples_;
}

std::uint64_t StreamAccounting::host_dropped_samples() const noexcept
{
    return host_dropped_samples_;
}

bool StreamAccounting::discontinuity_observed() const noexcept
{
    return device_.dropped_samples != 0 || host_dropped_samples_ != 0
        || (device_.flags
            & (AIRSPY_ONE_STREAM_DEVICE_LOSS | AIRSPY_ONE_STREAM_USB_ERROR))
            != 0;
}

} // namespace airspy::driver
