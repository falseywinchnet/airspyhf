#ifndef AIRSPY_DRIVER_BUFFER_POOL_HPP
#define AIRSPY_DRIVER_BUFFER_POOL_HPP

#include "airspy_driver/transfer_lifecycle.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace airspy::driver {

inline constexpr std::size_t consumer_buffer_count = 8;
inline constexpr std::size_t total_data_buffer_count =
    usb_transfer_count + consumer_buffer_count;

using BufferId = std::uint8_t;

enum class QueueResult : std::uint8_t {
    accepted,
    dropped,
    invalid_transfer,
    corrupt_state
};

struct ConsumerBlock {
    BufferId buffer{0};
    std::uint64_t preceding_dropped_blocks{0};
};

class BufferPool {
public:
    BufferPool() noexcept;

    [[nodiscard]] QueueResult accept_completed_transfer(
        std::size_t transfer_slot) noexcept;
    [[nodiscard]] bool begin_consume(ConsumerBlock& block) noexcept;
    [[nodiscard]] bool finish_consume() noexcept;

    [[nodiscard]] BufferId transfer_buffer(std::size_t slot) const noexcept;
    [[nodiscard]] std::size_t next_accept_slot() const noexcept;
    [[nodiscard]] ConsumerBlock consumer_slot(std::size_t slot) const noexcept;
    [[nodiscard]] std::size_t queued() const noexcept;
    [[nodiscard]] std::size_t high_watermark() const noexcept;
    [[nodiscard]] std::uint64_t pending_dropped_blocks() const noexcept;
    [[nodiscard]] std::uint64_t total_dropped_blocks() const noexcept;
    [[nodiscard]] bool ownership_is_unique() const noexcept;

private:
    enum class ConsumerSlotState : std::uint8_t {
        free,
        queued,
        processing
    };

    struct ConsumerSlot {
        BufferId buffer{0};
        std::uint64_t preceding_dropped_blocks{0};
        ConsumerSlotState state{ConsumerSlotState::free};
    };

    std::array<BufferId, usb_transfer_count> transfer_buffers_{};
    std::array<ConsumerSlot, consumer_buffer_count> consumer_slots_{};
    std::size_t head_{0};
    std::size_t tail_{0};
    std::size_t queued_{0};
    std::size_t high_watermark_{0};
    std::uint64_t pending_dropped_blocks_{0};
    std::uint64_t total_dropped_blocks_{0};
};

} // namespace airspy::driver

#endif
