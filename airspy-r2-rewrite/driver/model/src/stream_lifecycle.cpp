#include "airspy_driver/stream_lifecycle.hpp"

namespace airspy::driver {

StreamState StreamLifecycle::state() const noexcept
{
    return state_;
}

std::uint32_t StreamLifecycle::generation() const noexcept
{
    return generation_;
}

std::uint32_t StreamLifecycle::illegal_transitions() const noexcept
{
    return illegal_transitions_;
}

TransitionResult StreamLifecycle::change(
    const StreamState expected,
    const StreamState next) noexcept
{
    if (state_ != expected) {
        ++illegal_transitions_;
        return TransitionResult::illegal;
    }
    state_ = next;
    return TransitionResult::ok;
}

TransitionResult StreamLifecycle::open() noexcept
{
    return change(StreamState::closed, StreamState::open);
}

TransitionResult StreamLifecycle::begin_start() noexcept
{
    const TransitionResult result = change(StreamState::open, StreamState::starting);
    if (result == TransitionResult::ok) {
        ++generation_;
    }
    return result;
}

TransitionResult StreamLifecycle::complete_start() noexcept
{
    return change(StreamState::starting, StreamState::streaming);
}

TransitionResult StreamLifecycle::begin_stop() noexcept
{
    return change(StreamState::streaming, StreamState::stopping);
}

TransitionResult StreamLifecycle::complete_stop() noexcept
{
    return change(StreamState::stopping, StreamState::open);
}

TransitionResult StreamLifecycle::close() noexcept
{
    return change(StreamState::open, StreamState::closed);
}

void StreamLifecycle::fail() noexcept
{
    state_ = StreamState::faulted;
}

} // namespace airspy::driver
