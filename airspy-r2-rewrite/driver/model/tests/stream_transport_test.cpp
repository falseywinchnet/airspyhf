#include "airspy_driver/buffer_pool.hpp"
#include "airspy_driver/completion_path.hpp"
#include "airspy_driver/prepared_start.hpp"
#include "airspy_driver/sample_conversion.hpp"
#include "airspy_driver/transfer_lifecycle.hpp"

#include <array>
#include <cassert>
#include <cstdint>
#include <vector>

namespace {

using airspy::driver::PreparedStartBackend;
using airspy::driver::ReceiverMode;

enum class Event {
    off,
    clear_halt,
    armed,
    receive,
    submit,
    cancel_and_drain
};

class FakeStartBackend final : public PreparedStartBackend {
public:
    bool off_ok{true};
    bool armed_ok{true};
    bool receive_ok{true};
    bool submit_ok{true};
    bool drain_ok{true};
    std::vector<Event> events;

    bool set_receiver_mode(const ReceiverMode mode) noexcept override
    {
        switch (mode) {
        case ReceiverMode::off:
            events.push_back(Event::off);
            return off_ok;
        case ReceiverMode::armed:
            events.push_back(Event::armed);
            return armed_ok;
        case ReceiverMode::receive:
            events.push_back(Event::receive);
            return receive_ok;
        }
        return false;
    }

    void clear_bulk_in_halt() noexcept override
    {
        events.push_back(Event::clear_halt);
    }

    bool submit_transfer_pool() noexcept override
    {
        events.push_back(Event::submit);
        return submit_ok;
    }

    bool cancel_and_drain_transfer_pool() noexcept override
    {
        events.push_back(Event::cancel_and_drain);
        return drain_ok;
    }
};

class FakeCompletionBackend final : public airspy::driver::CompletionBackend {
public:
    bool resubmit_ok{true};
    std::vector<Event> events;

    bool resubmit(std::size_t) noexcept override
    {
        events.push_back(Event::submit);
        return resubmit_ok;
    }

    void notify_consumer() noexcept override
    {
        events.push_back(Event::receive);
    }
};

} // namespace

int main()
{
    using namespace airspy::driver;

    {
        FakeStartBackend backend;
        const StartOutcome outcome = start_receiver(backend);
        assert(outcome.result == StartResult::ok);
        assert(outcome.path == StartPath::prepared);
        assert((backend.events == std::vector{
            Event::off,
            Event::clear_halt,
            Event::armed,
            Event::submit,
            Event::receive}));
    }
    {
        FakeStartBackend backend;
        backend.armed_ok = false;
        const StartOutcome outcome = start_receiver(backend);
        assert(outcome.result == StartResult::ok);
        assert(outcome.path == StartPath::legacy);
        assert((backend.events == std::vector{
            Event::off,
            Event::clear_halt,
            Event::armed,
            Event::receive,
            Event::submit}));
    }
    {
        FakeStartBackend backend;
        backend.submit_ok = false;
        const StartOutcome outcome = start_receiver(backend);
        assert(outcome.result == StartResult::submit_failed);
        assert((backend.events == std::vector{
            Event::off,
            Event::clear_halt,
            Event::armed,
            Event::submit,
            Event::off,
            Event::cancel_and_drain}));
    }
    {
        FakeStartBackend backend;
        backend.receive_ok = false;
        const StartOutcome outcome = start_receiver(backend);
        assert(outcome.result == StartResult::receive_failed);
        assert((backend.events == std::vector{
            Event::off,
            Event::clear_halt,
            Event::armed,
            Event::submit,
            Event::receive,
            Event::off,
            Event::cancel_and_drain}));
    }
    {
        TransferLedger ledger;
        assert(ledger.begin_generation(7));
        for (std::size_t slot = 0; slot < usb_transfer_count; ++slot) {
            assert(ledger.submitted(slot, 7) == TransferObservation::ok);
        }
        assert(ledger.pending() == usb_transfer_count);
        assert(ledger.begin_streaming_observation());
        assert(ledger.minimum_pending() == usb_transfer_count);
        assert(!ledger.can_release_storage());
        assert(
            ledger.completed(0, 6, TransferCompletion::full)
            == TransferObservation::stale_generation);
        assert(
            ledger.completed(0, 7, TransferCompletion::full)
            == TransferObservation::ok);
        assert(ledger.pending() == usb_transfer_count - 1);
        assert(ledger.minimum_pending() == usb_transfer_count - 1);
        assert(
            ledger.submitted(0, 7)
            == TransferObservation::ok);
        assert(ledger.pending() == usb_transfer_count);

        ledger.end_streaming_observation();
        ledger.request_cancel_all(7);
        for (std::size_t slot = 0; slot < usb_transfer_count; ++slot) {
            assert(
                ledger.completed(slot, 7, TransferCompletion::cancelled)
                == TransferObservation::ok);
        }
        assert(ledger.pending() == 0);
        assert(ledger.terminal() == usb_transfer_count);
        assert(ledger.can_release_storage());
        ledger.release_completed();
        assert(ledger.terminal() == 0);
        assert(ledger.begin_generation(8));
    }
    {
        TransferLedger transfers;
        BufferPool buffers;
        FakeCompletionBackend backend;
        assert(transfers.begin_generation(11));
        for (std::size_t slot = 0; slot < usb_transfer_count; ++slot) {
            assert(
                transfers.submitted(slot, 11)
                == TransferObservation::ok);
        }
        assert(transfers.begin_streaming_observation());
        assert(
            handle_full_completion(transfers, buffers, backend, 0, 11)
            == FullCompletionResult::resubmitted_and_queued);
        assert((backend.events == std::vector{Event::submit, Event::receive}));
        assert(transfers.pending() == usb_transfer_count);
        assert(buffers.queued() == 1);
    }
    {
        TransferLedger transfers;
        BufferPool buffers;
        FakeCompletionBackend backend;
        assert(transfers.begin_generation(12));
        for (std::size_t slot = 0; slot < usb_transfer_count; ++slot) {
            assert(
                transfers.submitted(slot, 12)
                == TransferObservation::ok);
        }
        for (std::size_t slot = 0; slot < consumer_buffer_count; ++slot) {
            assert(
                buffers.accept_completed_transfer(slot)
                == QueueResult::accepted);
        }
        assert(
            handle_full_completion(
                transfers,
                buffers,
                backend,
                consumer_buffer_count,
                12)
            == FullCompletionResult::resubmitted_and_dropped);
        assert((backend.events == std::vector{Event::submit}));
        assert(transfers.pending() == usb_transfer_count);
        assert(buffers.total_dropped_blocks() == 1);
    }
    {
        TransferLedger transfers;
        BufferPool buffers;
        FakeCompletionBackend backend;
        backend.resubmit_ok = false;
        assert(transfers.begin_generation(13));
        assert(
            transfers.submitted(0, 13)
            == TransferObservation::ok);
        assert(
            handle_full_completion(transfers, buffers, backend, 0, 13)
            == FullCompletionResult::resubmit_failed);
        assert((backend.events == std::vector{Event::submit}));
        assert(transfers.pending() == 0);
        assert(transfers.can_release_storage());
    }
    {
        BufferPool buffers;
        assert(buffers.ownership_is_unique());
        for (std::size_t slot = 0; slot < consumer_buffer_count; ++slot) {
            assert(
                buffers.accept_completed_transfer(slot)
                == QueueResult::accepted);
        }
        assert(buffers.queued() == consumer_buffer_count);
        assert(buffers.high_watermark() == consumer_buffer_count);
        assert(
            buffers.accept_completed_transfer(consumer_buffer_count)
            == QueueResult::dropped);
        assert(buffers.pending_dropped_blocks() == 1);
        assert(buffers.total_dropped_blocks() == 1);

        ConsumerBlock block;
        assert(buffers.begin_consume(block));
        assert(block.preceding_dropped_blocks == 0);
        assert(buffers.finish_consume());
        assert(
            buffers.accept_completed_transfer(consumer_buffer_count)
            == QueueResult::accepted);
        assert(buffers.pending_dropped_blocks() == 0);

        for (std::size_t index = 0; index < consumer_buffer_count; ++index) {
            assert(buffers.begin_consume(block));
            if (index + 1 == consumer_buffer_count) {
                assert(block.preceding_dropped_blocks == 1);
            }
            assert(buffers.finish_consume());
        }
        assert(buffers.queued() == 0);
        assert(buffers.ownership_is_unique());
    }
    {
        constexpr std::array<std::uint16_t, 5> input{
            0, 1, 2047, 2048, 4095};
        std::array<std::int16_t, input.size()> integer_output{};
        std::array<float, input.size()> float_output{};
        assert(
            convert_u12_to_i16(input, integer_output)
            == ConversionResult::ok);
        assert((integer_output == std::array<std::int16_t, input.size()>{
            -32768, -32752, -16, 0, 32752}));
        assert(
            convert_u12_to_f32(input, float_output)
            == ConversionResult::ok);
        assert(float_output.front() == -1.0F);
        assert(float_output[3] == 0.0F);
        assert(float_output.back() == 2047.0F / 2048.0F);
    }
    return 0;
}
