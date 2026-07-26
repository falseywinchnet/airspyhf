#include "airspy_driver/transfer_lifecycle.hpp"

#include <cassert>

namespace airspy::driver {

bool TransferLedger::begin_generation(const std::uint32_t generation) noexcept
{
    if (!can_release_storage() || generation == 0) {
        return false;
    }
    for (TransferSlot& slot : slots_) {
        slot = {};
        slot.generation = generation;
    }
    generation_ = generation;
    pending_ = 0;
    minimum_pending_ = 0;
    terminal_ = 0;
    observing_stream_ = false;
    return true;
}

TransferObservation TransferLedger::submitted(
    const std::size_t index,
    const std::uint32_t generation) noexcept
{
    if (index >= slots_.size()) {
        return TransferObservation::invalid_slot;
    }
    if (generation != generation_) {
        return TransferObservation::stale_generation;
    }
    TransferSlot& transfer = slots_[index];
    if (transfer.generation != generation
        || (transfer.state != TransferState::idle
            && transfer.state != TransferState::completed)) {
        return TransferObservation::illegal_state;
    }
    if (transfer.state == TransferState::completed) {
        assert(terminal_ != 0);
        --terminal_;
    }
    transfer.state = TransferState::submitted;
    ++pending_;
    observe_pending();
    return TransferObservation::ok;
}

TransferObservation TransferLedger::request_cancel(
    const std::size_t index,
    const std::uint32_t generation) noexcept
{
    if (index >= slots_.size()) {
        return TransferObservation::invalid_slot;
    }
    if (generation != generation_) {
        return TransferObservation::stale_generation;
    }
    TransferSlot& transfer = slots_[index];
    if (transfer.generation != generation
        || transfer.state != TransferState::submitted) {
        return TransferObservation::illegal_state;
    }
    transfer.state = TransferState::cancel_requested;
    return TransferObservation::ok;
}

TransferObservation TransferLedger::completed(
    const std::size_t index,
    const std::uint32_t generation,
    const TransferCompletion completion) noexcept
{
    if (index >= slots_.size()) {
        return TransferObservation::invalid_slot;
    }
    if (generation != generation_) {
        return TransferObservation::stale_generation;
    }
    TransferSlot& transfer = slots_[index];
    if (transfer.generation != generation
        || (transfer.state != TransferState::submitted
            && transfer.state != TransferState::cancel_requested)) {
        return TransferObservation::illegal_state;
    }
    transfer.state = TransferState::completed;
    transfer.completion = completion;
    assert(pending_ != 0);
    --pending_;
    ++terminal_;
    observe_pending();
    return TransferObservation::ok;
}

void TransferLedger::request_cancel_all(const std::uint32_t generation) noexcept
{
    if (generation != generation_) {
        return;
    }
    for (TransferSlot& transfer : slots_) {
        if (transfer.state == TransferState::submitted) {
            transfer.state = TransferState::cancel_requested;
        }
    }
}

bool TransferLedger::begin_streaming_observation() noexcept
{
    if (observing_stream_ || pending_ == 0) {
        return false;
    }
    observing_stream_ = true;
    minimum_pending_ = pending_;
    return true;
}

void TransferLedger::end_streaming_observation() noexcept
{
    observing_stream_ = false;
}

void TransferLedger::release_completed() noexcept
{
    if (pending_ != 0) {
        return;
    }
    for (TransferSlot& transfer : slots_) {
        transfer.state = TransferState::idle;
    }
    terminal_ = 0;
}

const TransferSlot& TransferLedger::slot(const std::size_t index) const noexcept
{
    assert(index < slots_.size());
    return slots_[index];
}

std::uint32_t TransferLedger::generation() const noexcept
{
    return generation_;
}

std::size_t TransferLedger::pending() const noexcept
{
    return pending_;
}

std::size_t TransferLedger::minimum_pending() const noexcept
{
    return minimum_pending_;
}

std::size_t TransferLedger::terminal() const noexcept
{
    return terminal_;
}

bool TransferLedger::can_release_storage() const noexcept
{
    return pending_ == 0;
}

void TransferLedger::observe_pending() noexcept
{
    if (observing_stream_ && pending_ < minimum_pending_) {
        minimum_pending_ = pending_;
    }
}

} // namespace airspy::driver
