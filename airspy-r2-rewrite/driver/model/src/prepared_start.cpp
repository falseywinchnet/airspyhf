#include "airspy_driver/prepared_start.hpp"

namespace airspy::driver {
namespace {

StartResult unwind_started_pool(PreparedStartBackend& backend) noexcept
{
    const bool stopped = backend.set_receiver_mode(ReceiverMode::off);
    const bool drained = backend.cancel_and_drain_transfer_pool();
    return stopped && drained ? StartResult::ok : StartResult::cleanup_failed;
}

} // namespace

StartOutcome start_receiver(PreparedStartBackend& backend) noexcept
{
    if (!backend.set_receiver_mode(ReceiverMode::off)) {
        return {StartResult::stop_failed, StartPath::none};
    }
    backend.clear_bulk_in_halt();

    if (backend.set_receiver_mode(ReceiverMode::armed)) {
        constexpr StartPath path = StartPath::prepared;
        if (!backend.submit_transfer_pool()) {
            const StartResult cleanup = unwind_started_pool(backend);
            return {
                cleanup == StartResult::ok
                    ? StartResult::submit_failed
                    : StartResult::cleanup_failed,
                path};
        }
        if (!backend.set_receiver_mode(ReceiverMode::receive)) {
            const StartResult cleanup = unwind_started_pool(backend);
            return {
                cleanup == StartResult::ok
                    ? StartResult::receive_failed
                    : StartResult::cleanup_failed,
                path};
        }
        return {StartResult::ok, path};
    }

    constexpr StartPath path = StartPath::legacy;
    if (!backend.set_receiver_mode(ReceiverMode::receive)) {
        return {StartResult::receive_failed, path};
    }
    if (!backend.submit_transfer_pool()) {
        const StartResult cleanup = unwind_started_pool(backend);
        return {
            cleanup == StartResult::ok
                ? StartResult::submit_failed
                : StartResult::cleanup_failed,
            path};
    }
    return {StartResult::ok, path};
}

} // namespace airspy::driver
