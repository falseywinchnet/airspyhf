#ifndef AIRSPY_DRIVER_STREAM_ACCOUNTING_HPP
#define AIRSPY_DRIVER_STREAM_ACCOUNTING_HPP

#include "airspy_one/protocol.h"

#include <cstdint>

namespace airspy::driver {

enum class StatusObservation {
    ok,
    stale_generation,
    regressed,
    inconsistent
};

class StreamAccounting {
public:
    void begin_generation(std::uint32_t generation) noexcept;

    [[nodiscard]] StatusObservation observe_device(
        const airspy_one_stream_status& status) noexcept;
    void record_host_delivery(std::uint64_t samples) noexcept;
    void record_host_discontinuity(std::uint64_t samples) noexcept;

    [[nodiscard]] std::uint32_t generation() const noexcept;
    [[nodiscard]] const airspy_one_stream_status& device() const noexcept;
    [[nodiscard]] std::uint64_t host_delivered_samples() const noexcept;
    [[nodiscard]] std::uint64_t host_dropped_samples() const noexcept;
    [[nodiscard]] bool discontinuity_observed() const noexcept;

private:
    std::uint32_t generation_{0};
    airspy_one_stream_status device_{};
    std::uint64_t host_delivered_samples_{0};
    std::uint64_t host_dropped_samples_{0};
};

} // namespace airspy::driver

#endif
