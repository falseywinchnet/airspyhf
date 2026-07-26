#include "airspy_driver/completion_path.hpp"

namespace airspy::driver {

FullCompletionResult handle_full_completion(
    TransferLedger& transfers,
    BufferPool& buffers,
    CompletionBackend& backend,
    const std::size_t transfer_slot,
    const std::uint32_t generation) noexcept
{
    const TransferObservation completion = transfers.completed(
        transfer_slot,
        generation,
        TransferCompletion::full);
    if (completion == TransferObservation::stale_generation) {
        return FullCompletionResult::stale_generation;
    }
    if (completion != TransferObservation::ok) {
        return FullCompletionResult::illegal_transfer_state;
    }

    const QueueResult queue = buffers.accept_completed_transfer(transfer_slot);
    if (queue == QueueResult::invalid_transfer
        || queue == QueueResult::corrupt_state) {
        return FullCompletionResult::buffer_ownership_fault;
    }

    /*
     * Nothing that can wake or run the consumer belongs above this point.
     * Once ownership has moved, restore the endpoint's pending depth before
     * making the completed block visible to expensive host work.
     */
    if (!backend.resubmit(transfer_slot)) {
        return FullCompletionResult::resubmit_failed;
    }
    if (transfers.submitted(transfer_slot, generation)
        != TransferObservation::ok) {
        return FullCompletionResult::illegal_transfer_state;
    }

    if (queue == QueueResult::accepted) {
        backend.notify_consumer();
        return FullCompletionResult::resubmitted_and_queued;
    }
    return FullCompletionResult::resubmitted_and_dropped;
}

} // namespace airspy::driver
