#ifndef AIRSPY_DRIVER_TRANSFER_LIFECYCLE_HPP
#define AIRSPY_DRIVER_TRANSFER_LIFECYCLE_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace airspy::driver {

inline constexpr std::size_t usb_transfer_count = 16;

enum class TransferState : std::uint8_t {
    idle,
    submitted,
    cancel_requested,
    completed
};

enum class TransferCompletion : std::uint8_t {
    full,
    short_transfer,
    cancelled,
    stalled,
    disconnected,
    overflow,
    backend_error
};

enum class TransferObservation : std::uint8_t {
    ok,
    invalid_slot,
    stale_generation,
    illegal_state
};

struct TransferSlot {
    TransferState state{TransferState::idle};
    std::uint32_t generation{0};
    TransferCompletion completion{TransferCompletion::backend_error};
};

class TransferLedger {
public:
    [[nodiscard]] bool begin_generation(std::uint32_t generation) noexcept;

    [[nodiscard]] TransferObservation submitted(
        std::size_t slot,
        std::uint32_t generation) noexcept;
    [[nodiscard]] TransferObservation request_cancel(
        std::size_t slot,
        std::uint32_t generation) noexcept;
    [[nodiscard]] TransferObservation completed(
        std::size_t slot,
        std::uint32_t generation,
        TransferCompletion completion) noexcept;

    [[nodiscard]] bool begin_streaming_observation() noexcept;
    void end_streaming_observation() noexcept;
    void request_cancel_all(std::uint32_t generation) noexcept;
    void release_completed() noexcept;

    [[nodiscard]] const TransferSlot& slot(std::size_t index) const noexcept;
    [[nodiscard]] std::uint32_t generation() const noexcept;
    [[nodiscard]] std::size_t pending() const noexcept;
    [[nodiscard]] std::size_t minimum_pending() const noexcept;
    [[nodiscard]] std::size_t terminal() const noexcept;
    [[nodiscard]] bool can_release_storage() const noexcept;

private:
    void observe_pending() noexcept;

    std::array<TransferSlot, usb_transfer_count> slots_{};
    std::uint32_t generation_{0};
    std::size_t pending_{0};
    std::size_t minimum_pending_{0};
    std::size_t terminal_{0};
    bool observing_stream_{false};
};

} // namespace airspy::driver

#endif
