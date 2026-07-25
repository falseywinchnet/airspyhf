#ifndef AIRSPY_DRIVER_STREAM_LIFECYCLE_HPP
#define AIRSPY_DRIVER_STREAM_LIFECYCLE_HPP

#include <cstdint>

namespace airspy::driver {

enum class StreamState {
    closed,
    open,
    starting,
    streaming,
    stopping,
    faulted
};

enum class TransitionResult {
    ok,
    illegal
};

class StreamLifecycle {
public:
    [[nodiscard]] StreamState state() const noexcept;
    [[nodiscard]] std::uint32_t generation() const noexcept;
    [[nodiscard]] std::uint32_t illegal_transitions() const noexcept;

    TransitionResult open() noexcept;
    TransitionResult begin_start() noexcept;
    TransitionResult complete_start() noexcept;
    TransitionResult begin_stop() noexcept;
    TransitionResult complete_stop() noexcept;
    TransitionResult close() noexcept;
    void fail() noexcept;

private:
    TransitionResult change(StreamState expected, StreamState next) noexcept;

    StreamState state_{StreamState::closed};
    std::uint32_t generation_{0};
    std::uint32_t illegal_transitions_{0};
};

} // namespace airspy::driver

#endif
