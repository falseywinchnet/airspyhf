#include "airspy_driver/buffer_pool.hpp"

#include <array>
#include <cassert>
#include <utility>

namespace airspy::driver {

BufferPool::BufferPool() noexcept
{
    for (std::size_t index = 0; index < transfer_buffers_.size(); ++index) {
        transfer_buffers_[index] = static_cast<BufferId>(index);
    }
    for (std::size_t index = 0; index < consumer_slots_.size(); ++index) {
        consumer_slots_[index].buffer =
            static_cast<BufferId>(usb_transfer_count + index);
    }
    assert(ownership_is_unique());
}

QueueResult BufferPool::accept_completed_transfer(
    const std::size_t transfer_slot) noexcept
{
    if (transfer_slot >= transfer_buffers_.size()) {
        return QueueResult::invalid_transfer;
    }
    if (queued_ == consumer_slots_.size()) {
        ++pending_dropped_blocks_;
        ++total_dropped_blocks_;
        return QueueResult::dropped;
    }

    ConsumerSlot& destination = consumer_slots_[head_];
    if (destination.state != ConsumerSlotState::free) {
        return QueueResult::corrupt_state;
    }
    std::swap(destination.buffer, transfer_buffers_[transfer_slot]);
    destination.preceding_dropped_blocks = pending_dropped_blocks_;
    destination.state = ConsumerSlotState::queued;
    pending_dropped_blocks_ = 0;
    head_ = (head_ + 1) % consumer_slots_.size();
    ++queued_;
    if (queued_ > high_watermark_) {
        high_watermark_ = queued_;
    }
    assert(ownership_is_unique());
    return QueueResult::accepted;
}

bool BufferPool::begin_consume(ConsumerBlock& block) noexcept
{
    if (queued_ == 0) {
        return false;
    }
    ConsumerSlot& source = consumer_slots_[tail_];
    if (source.state != ConsumerSlotState::queued) {
        return false;
    }
    source.state = ConsumerSlotState::processing;
    block.buffer = source.buffer;
    block.preceding_dropped_blocks = source.preceding_dropped_blocks;
    return true;
}

bool BufferPool::finish_consume() noexcept
{
    if (queued_ == 0) {
        return false;
    }
    ConsumerSlot& source = consumer_slots_[tail_];
    if (source.state != ConsumerSlotState::processing) {
        return false;
    }
    source.preceding_dropped_blocks = 0;
    source.state = ConsumerSlotState::free;
    tail_ = (tail_ + 1) % consumer_slots_.size();
    --queued_;
    assert(ownership_is_unique());
    return true;
}

BufferId BufferPool::transfer_buffer(const std::size_t slot) const noexcept
{
    assert(slot < transfer_buffers_.size());
    return transfer_buffers_[slot];
}

std::size_t BufferPool::next_accept_slot() const noexcept
{
    return head_;
}

ConsumerBlock BufferPool::consumer_slot(const std::size_t slot) const noexcept
{
    assert(slot < consumer_slots_.size());
    return {
        consumer_slots_[slot].buffer,
        consumer_slots_[slot].preceding_dropped_blocks};
}

std::size_t BufferPool::queued() const noexcept
{
    return queued_;
}

std::size_t BufferPool::high_watermark() const noexcept
{
    return high_watermark_;
}

std::uint64_t BufferPool::pending_dropped_blocks() const noexcept
{
    return pending_dropped_blocks_;
}

std::uint64_t BufferPool::total_dropped_blocks() const noexcept
{
    return total_dropped_blocks_;
}

bool BufferPool::ownership_is_unique() const noexcept
{
    std::array<bool, total_data_buffer_count> seen{};
    for (const BufferId buffer : transfer_buffers_) {
        if (buffer >= seen.size() || seen[buffer]) {
            return false;
        }
        seen[buffer] = true;
    }
    for (const ConsumerSlot& slot : consumer_slots_) {
        if (slot.buffer >= seen.size() || seen[slot.buffer]) {
            return false;
        }
        seen[slot.buffer] = true;
    }
    for (const bool present : seen) {
        if (!present) {
            return false;
        }
    }
    return true;
}

} // namespace airspy::driver
