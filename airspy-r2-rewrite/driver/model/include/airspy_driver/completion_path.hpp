#ifndef AIRSPY_DRIVER_COMPLETION_PATH_HPP
#define AIRSPY_DRIVER_COMPLETION_PATH_HPP

#include "airspy_driver/buffer_pool.hpp"
#include "airspy_driver/transfer_lifecycle.hpp"

#include <cstddef>
#include <cstdint>

namespace airspy::driver {

enum class FullCompletionResult : std::uint8_t {
    resubmitted_and_queued,
    resubmitted_and_dropped,
    stale_generation,
    illegal_transfer_state,
    buffer_ownership_fault,
    resubmit_failed
};

class CompletionBackend {
public:
    virtual ~CompletionBackend() = default;

    virtual bool resubmit(std::size_t transfer_slot) noexcept = 0;
    virtual void notify_consumer() noexcept = 0;
};

[[nodiscard]] FullCompletionResult handle_full_completion(
    TransferLedger& transfers,
    BufferPool& buffers,
    CompletionBackend& backend,
    std::size_t transfer_slot,
    std::uint32_t generation) noexcept;

} // namespace airspy::driver

#endif
