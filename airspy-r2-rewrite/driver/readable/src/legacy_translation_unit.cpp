/*
 * Transitional readable-driver build.
 *
 * The field-compatible implementation is compiled as C++ here while
 * `current/` continues to compile the same source as C. This target is a
 * behavioral bridge, not the destination architecture. Internal functions are
 * extracted from the included parity oracle one owner at a time; exported C
 * symbols remain governed by airspy.h.
 */
#include "airspy.h"
#include "libusb.h"

#include "airspy_driver/buffer_pool.hpp"
#include "airspy_driver/legacy_unpack.hpp"
#include "airspy_driver/sample_conversion.hpp"
#include "airspy_driver/transfer_lifecycle.hpp"

#include <array>
#include <atomic>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <limits>
#include <mutex>
#include <new>
#include <span>

static bool readable_runtime_create(struct airspy_device* device);
static void readable_runtime_destroy(struct airspy_device* device);
static bool readable_runtime_prepare_free(struct airspy_device* device);
static void readable_runtime_rebind(struct airspy_device* device);
static int readable_prepare_transfers(
    struct airspy_device* device,
    uint_fast8_t endpoint,
    libusb_transfer_cb_fn callback);
static bool readable_begin_consume(
    struct airspy_device* device,
    std::uint16_t** samples,
    std::uint64_t* dropped_buffers);
static bool readable_finish_consume(struct airspy_device* device);
static void readable_libusb_transfer_callback(libusb_transfer* transfer);
static void* readable_transfer_threadproc(void* argument);
static int readable_kill_io_threads(struct airspy_device* device);
static int readable_create_io_threads(
    struct airspy_device* device,
    airspy_sample_block_cb_fn callback);
static bool readable_stream_should_run(struct airspy_device* device);
static bool readable_stream_is_running(struct airspy_device* device);
static void readable_stream_request_stop(struct airspy_device* device);
static void readable_stream_mark_stopped(struct airspy_device* device);
static int readable_start_rx(
    struct airspy_device* device,
    airspy_sample_block_cb_fn callback,
    void* context);
static int readable_stop_rx(struct airspy_device* device);
static int readable_close(struct airspy_device* device);

namespace {

void convert_samples_int16(
    std::uint16_t* const source,
    std::int16_t* const destination,
    const int count)
{
    (void)airspy::driver::convert_u12_to_i16(
        std::span<const std::uint16_t>(
            source,
            static_cast<std::size_t>(count)),
        std::span<std::int16_t>(
            destination,
            static_cast<std::size_t>(count)));
}

void convert_samples_float(
    std::uint16_t* const source,
    float* const destination,
    const int count)
{
    (void)airspy::driver::convert_u12_to_f32(
        std::span<const std::uint16_t>(
            source,
            static_cast<std::size_t>(count)),
        std::span<float>(
            destination,
            static_cast<std::size_t>(count)));
}

void unpack_samples(
    std::uint32_t* const input,
    std::uint16_t* const output,
    const int output_length)
{
    const std::size_t output_samples =
        static_cast<std::size_t>(output_length);
    const std::size_t input_bytes = (output_samples / 8U) * 12U;
    (void)airspy::driver::unpack_legacy_u12(
        std::span<const std::uint8_t>(
            reinterpret_cast<const std::uint8_t*>(input),
            input_bytes),
        std::span<std::uint16_t>(output, output_samples));
}

void unpack_samples_int16(
    std::uint32_t* const input,
    std::int16_t* const output,
    const int output_length)
{
    const std::size_t output_samples =
        static_cast<std::size_t>(output_length);
    const std::size_t input_bytes = (output_samples / 8U) * 12U;
    (void)airspy::driver::unpack_legacy_u12_to_i16(
        std::span<const std::uint8_t>(
            reinterpret_cast<const std::uint8_t*>(input),
            input_bytes),
        std::span<std::int16_t>(output, output_samples));
}

void unpack_samples_float(
    std::uint32_t* const input,
    float* const output,
    const int output_length)
{
    const std::size_t output_samples =
        static_cast<std::size_t>(output_length);
    const std::size_t input_bytes = (output_samples / 8U) * 12U;
    (void)airspy::driver::unpack_legacy_u12_to_f32(
        std::span<const std::uint8_t>(
            reinterpret_cast<const std::uint8_t*>(input),
            input_bytes),
        std::span<float>(output, output_samples));
}

} // namespace

#define AIRSPY_READABLE_RUNTIME
#define AIRSPY_READABLE_EXTERNAL_SAMPLE_CONVERSION
#define AIRSPY_READABLE_EXTERNAL_PACKED_UNPACK
#define AIRSPY_READABLE_EXTERNAL_PACKED_CONVERSION
#include "../../current/libairspy/src/airspy.c"

namespace {

using airspy::driver::BufferId;
using airspy::driver::BufferPool;
using airspy::driver::ConsumerBlock;
using airspy::driver::QueueResult;
using airspy::driver::TransferCompletion;
using airspy::driver::TransferLedger;
using airspy::driver::TransferObservation;
using airspy::driver::usb_transfer_count;
using airspy::driver::total_data_buffer_count;

class ReadableRuntime;

struct TransferContext {
    ReadableRuntime* runtime{nullptr};
    std::size_t slot{0};
};

TransferCompletion classify_completion(const libusb_transfer& transfer) noexcept
{
    switch (transfer.status) {
    case LIBUSB_TRANSFER_COMPLETED:
        return transfer.actual_length == transfer.length
            ? TransferCompletion::full
            : TransferCompletion::short_transfer;
    case LIBUSB_TRANSFER_CANCELLED:
        return TransferCompletion::cancelled;
    case LIBUSB_TRANSFER_STALL:
        return TransferCompletion::stalled;
    case LIBUSB_TRANSFER_NO_DEVICE:
        return TransferCompletion::disconnected;
    case LIBUSB_TRANSFER_OVERFLOW:
        return TransferCompletion::overflow;
    case LIBUSB_TRANSFER_ERROR:
    case LIBUSB_TRANSFER_TIMED_OUT:
        return TransferCompletion::backend_error;
    }
    return TransferCompletion::backend_error;
}

class ReadableRuntime {
public:
    explicit ReadableRuntime(airspy_device_t& device) noexcept
        : device_(device)
    {
        for (std::size_t slot = 0; slot < contexts_.size(); ++slot) {
            contexts_[slot] = {this, slot};
        }
    }

    bool bind_new_allocation() noexcept
    {
        std::lock_guard lock(transfer_mutex_);
        if (ledger_.pending() != 0 || device_.transfers == nullptr) {
            return false;
        }
        for (std::size_t slot = 0; slot < usb_transfer_count; ++slot) {
            if (device_.transfers[slot] == nullptr
                || device_.transfers[slot]->buffer == nullptr) {
                return false;
            }
            allocation_[slot] = device_.transfers[slot]->buffer;
        }
        for (std::size_t slot = 0;
             slot < airspy::driver::consumer_buffer_count;
             ++slot) {
            if (device_.received_samples_queue[slot] == nullptr) {
                return false;
            }
            allocation_[usb_transfer_count + slot] =
                reinterpret_cast<std::uint8_t*>(
                    device_.received_samples_queue[slot]);
        }
        allocation_bound_ = true;
        reset_buffer_ownership_locked();
        return true;
    }

    bool prepare_allocation_for_free() noexcept
    {
        std::lock_guard lock(transfer_mutex_);
        if (ledger_.pending() != 0
            || transfer_worker_running_.load(std::memory_order_acquire)
            || consumer_worker_running_.load(std::memory_order_acquire)) {
            return false;
        }
        if (allocation_bound_) {
            restore_allocation_layout_locked();
            allocation_bound_ = false;
        }
        return true;
    }

    bool begin_start() noexcept
    {
        std::lock_guard lock(transfer_mutex_);
        if (!allocation_bound_ || ledger_.pending() != 0
            || transfer_worker_running_.load(std::memory_order_acquire)
            || consumer_worker_running_.load(std::memory_order_acquire)) {
            return false;
        }
        reset_buffer_ownership_locked();
        stop_requested_.store(false, std::memory_order_release);
        running_.store(true, std::memory_order_release);
        faulted_.store(false, std::memory_order_release);
        return true;
    }

    int submit_pool(
        const std::uint_fast8_t endpoint,
        const libusb_transfer_cb_fn callback) noexcept
    {
        {
            std::lock_guard lock(transfer_mutex_);
            if (ledger_.pending() != 0) {
                return AIRSPY_ERROR_BUSY;
            }
            ++generation_;
            if (generation_ == 0) {
                ++generation_;
            }
            if (!ledger_.begin_generation(generation_)) {
                return AIRSPY_ERROR_OTHER;
            }

            for (std::size_t slot = 0; slot < usb_transfer_count; ++slot) {
                libusb_transfer& transfer = *device_.transfers[slot];
                transfer.endpoint = static_cast<unsigned char>(endpoint);
                transfer.callback = callback;
                transfer.user_data = &contexts_[slot];
                transfer.buffer = allocation_[buffers_.transfer_buffer(slot)];

                const int result = libusb_submit_transfer(&transfer);
                if (result != LIBUSB_SUCCESS) {
                    request_stop_locked();
                    cancel_all_locked();
                    break;
                }
                if (ledger_.submitted(slot, generation_)
                    != TransferObservation::ok) {
                    request_stop_locked();
                    cancel_all_locked();
                    break;
                }
            }
            if (ledger_.pending() == usb_transfer_count) {
                (void)ledger_.begin_streaming_observation();
                return AIRSPY_SUCCESS;
            }
        }

        drain_in_calling_thread();
        return AIRSPY_ERROR_LIBUSB;
    }

    void on_completion(
        const std::size_t slot,
        libusb_transfer& transfer) noexcept
    {
        bool notify_consumer = false;
        std::unique_lock transfer_lock(transfer_mutex_);
        const TransferCompletion completion = classify_completion(transfer);
        const TransferObservation observed =
            ledger_.completed(slot, generation_, completion);
        if (observed != TransferObservation::ok) {
            request_stop_locked();
            cancel_all_locked();
            return;
        }

        if (completion != TransferCompletion::full
            || !should_run_locked()) {
            if (completion != TransferCompletion::cancelled
                && stop_requested_.load(std::memory_order_relaxed) == false) {
                faulted_.store(true, std::memory_order_release);
                request_stop_locked();
                cancel_all_locked();
            }
            return;
        }

        QueueResult queue_result;
        pthread_mutex_lock(&device_.consumer_mp);
        queue_result = buffers_.accept_completed_transfer(slot);
        if (queue_result == QueueResult::accepted) {
            notify_consumer = true;
        } else if (queue_result != QueueResult::dropped) {
            pthread_mutex_unlock(&device_.consumer_mp);
            faulted_.store(true, std::memory_order_release);
            request_stop_locked();
            cancel_all_locked();
            return;
        }
        transfer.buffer = allocation_[buffers_.transfer_buffer(slot)];
        device_.received_buffer_count =
            static_cast<int>(buffers_.queued());
        pthread_mutex_unlock(&device_.consumer_mp);

        /*
         * The queue lock is deliberately gone before resubmission. The
         * consumer has not been signalled yet, so no intentional DSP/client
         * work is released ahead of restoring endpoint depth.
         */
        if (libusb_submit_transfer(&transfer) != LIBUSB_SUCCESS
            || ledger_.submitted(slot, generation_)
                != TransferObservation::ok) {
            faulted_.store(true, std::memory_order_release);
            request_stop_locked();
            cancel_all_locked();
            return;
        }
        transfer_lock.unlock();

        if (notify_consumer) {
            pthread_cond_signal(&device_.consumer_cv);
        }
    }

    bool begin_consume(
        std::uint16_t*& samples,
        std::uint64_t& dropped_buffers) noexcept
    {
        ConsumerBlock block;
        if (!buffers_.begin_consume(block)) {
            return false;
        }
        samples = reinterpret_cast<std::uint16_t*>(allocation_[block.buffer]);
        dropped_buffers = block.preceding_dropped_blocks;
        return true;
    }

    bool finish_consume() noexcept
    {
        if (!buffers_.finish_consume()) {
            return false;
        }
        device_.received_buffer_count = static_cast<int>(buffers_.queued());
        return true;
    }

    bool should_run() const noexcept
    {
        return running_.load(std::memory_order_acquire)
            && !stop_requested_.load(std::memory_order_acquire);
    }

    bool is_running() const noexcept
    {
        std::lock_guard lock(transfer_mutex_);
        return running_.load(std::memory_order_acquire)
            || ledger_.pending() != 0
            || transfer_worker_running_.load(std::memory_order_acquire)
            || consumer_worker_running_.load(std::memory_order_acquire);
    }

    void request_stop() noexcept
    {
        std::lock_guard lock(transfer_mutex_);
        request_stop_locked();
        cancel_all_locked();
    }

    void mark_stopped() noexcept
    {
        running_.store(false, std::memory_order_release);
    }

    void event_loop() noexcept
    {
        while (should_run()) {
            timeval timeout{0, 500000};
            const int result = libusb_handle_events_timeout_completed(
                device_.usb_context,
                &timeout,
                nullptr);
            if (result < 0 && result != LIBUSB_ERROR_INTERRUPTED) {
                faulted_.store(true, std::memory_order_release);
                request_stop();
            }
        }

        request_stop();
        while (pending() != 0) {
            timeval timeout{0, 50000};
            const int result = libusb_handle_events_timeout_completed(
                device_.usb_context,
                &timeout,
                nullptr);
            if (result == LIBUSB_ERROR_INTERRUPTED) {
                continue;
            }
            /*
             * Even after a backend error, active request storage cannot be
             * released. Keep giving libusb an event context until every
             * terminal callback has been observed.
             */
        }
        ledger_.end_streaming_observation();
        mark_stopped();
    }

    void drain_in_calling_thread() noexcept
    {
        request_stop();
        while (pending() != 0) {
            timeval timeout{0, 50000};
            (void)libusb_handle_events_timeout_completed(
                device_.usb_context,
                &timeout,
                nullptr);
        }
        ledger_.end_streaming_observation();
        mark_stopped();
    }

    std::size_t pending() noexcept
    {
        std::lock_guard lock(transfer_mutex_);
        return ledger_.pending();
    }

    bool faulted() const noexcept
    {
        return faulted_.load(std::memory_order_acquire);
    }

    std::recursive_mutex& lifecycle_mutex() noexcept
    {
        return lifecycle_mutex_;
    }

    bool transfer_worker_running() const noexcept
    {
        return transfer_worker_running_.load(std::memory_order_acquire);
    }

    bool consumer_worker_running() const noexcept
    {
        return consumer_worker_running_.load(std::memory_order_acquire);
    }

    void set_transfer_worker_running(const bool running) noexcept
    {
        transfer_worker_running_.store(running, std::memory_order_release);
    }

    void set_consumer_worker_running(const bool running) noexcept
    {
        consumer_worker_running_.store(running, std::memory_order_release);
    }

private:
    bool should_run_locked() const noexcept
    {
        return running_.load(std::memory_order_relaxed)
            && !stop_requested_.load(std::memory_order_relaxed);
    }

    void request_stop_locked() noexcept
    {
        stop_requested_.store(true, std::memory_order_release);
        running_.store(false, std::memory_order_release);
    }

    void cancel_all_locked() noexcept
    {
        ledger_.request_cancel_all(generation_);
        for (std::size_t slot = 0; slot < usb_transfer_count; ++slot) {
            if (ledger_.slot(slot).state
                == airspy::driver::TransferState::cancel_requested) {
                const int result =
                    libusb_cancel_transfer(device_.transfers[slot]);
                (void)result;
            }
        }
    }

    void restore_allocation_layout_locked() noexcept
    {
        for (std::size_t slot = 0; slot < usb_transfer_count; ++slot) {
            device_.transfers[slot]->buffer = allocation_[slot];
            device_.transfers[slot]->user_data = &contexts_[slot];
        }
        for (std::size_t slot = 0;
             slot < airspy::driver::consumer_buffer_count;
             ++slot) {
            device_.received_samples_queue[slot] =
                reinterpret_cast<std::uint16_t*>(
                    allocation_[usb_transfer_count + slot]);
        }
    }

    void reset_buffer_ownership_locked() noexcept
    {
        restore_allocation_layout_locked();
        buffers_ = BufferPool{};
        device_.received_samples_queue_head = 0;
        device_.received_samples_queue_tail = 0;
        device_.received_buffer_count = 0;
        device_.dropped_buffers = 0;
    }

    airspy_device_t& device_;
    std::recursive_mutex lifecycle_mutex_;
    mutable std::mutex transfer_mutex_;
    TransferLedger ledger_;
    BufferPool buffers_;
    std::array<std::uint8_t*, total_data_buffer_count> allocation_{};
    std::array<TransferContext, usb_transfer_count> contexts_{};
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
    std::atomic<bool> faulted_{false};
    std::atomic<bool> transfer_worker_running_{false};
    std::atomic<bool> consumer_worker_running_{false};
    std::uint32_t generation_{0};
    bool allocation_bound_{false};
};

ReadableRuntime* runtime(airspy_device_t* const device) noexcept
{
    return device == nullptr
        ? nullptr
        : static_cast<ReadableRuntime*>(device->readable_runtime);
}

} // namespace

static bool readable_runtime_create(airspy_device* const device)
{
    if (device == nullptr || device->readable_runtime != nullptr) {
        return false;
    }
    auto* const created = new (std::nothrow) ReadableRuntime(*device);
    if (created == nullptr) {
        return false;
    }
    device->readable_runtime = created;
    if (!created->bind_new_allocation()) {
        device->readable_runtime = nullptr;
        delete created;
        return false;
    }
    return true;
}

static void readable_runtime_destroy(airspy_device* const device)
{
    ReadableRuntime* const owned = runtime(device);
    if (device != nullptr) {
        device->readable_runtime = nullptr;
    }
    delete owned;
}

static bool readable_runtime_prepare_free(airspy_device* const device)
{
    ReadableRuntime* const owner = runtime(device);
    return owner == nullptr || owner->prepare_allocation_for_free();
}

static void readable_runtime_rebind(airspy_device* const device)
{
    ReadableRuntime* const owner = runtime(device);
    if (owner != nullptr && !owner->bind_new_allocation()) {
        readable_stream_request_stop(device);
    }
}

static int readable_prepare_transfers(
    airspy_device* const device,
    const uint_fast8_t endpoint,
    const libusb_transfer_cb_fn callback)
{
    ReadableRuntime* const owner = runtime(device);
    return owner == nullptr
        ? AIRSPY_ERROR_OTHER
        : owner->submit_pool(endpoint, callback);
}

static bool readable_begin_consume(
    airspy_device* const device,
    std::uint16_t** const samples,
    std::uint64_t* const dropped_buffers)
{
    ReadableRuntime* const owner = runtime(device);
    return owner != nullptr && samples != nullptr && dropped_buffers != nullptr
        && owner->begin_consume(*samples, *dropped_buffers);
}

static bool readable_finish_consume(airspy_device* const device)
{
    ReadableRuntime* const owner = runtime(device);
    return owner != nullptr && owner->finish_consume();
}

static void readable_libusb_transfer_callback(libusb_transfer* const transfer)
{
    if (transfer == nullptr || transfer->user_data == nullptr) {
        return;
    }
    auto* const context =
        static_cast<TransferContext*>(transfer->user_data);
    if (context->runtime != nullptr) {
        context->runtime->on_completion(context->slot, *transfer);
    }
}

static void* readable_transfer_threadproc(void* const argument)
{
    auto* const device = static_cast<airspy_device_t*>(argument);
    ReadableRuntime* const owner = runtime(device);
    if (owner != nullptr) {
        owner->event_loop();
    }
    return nullptr;
}

static bool readable_stream_should_run(airspy_device* const device)
{
    const ReadableRuntime* const owner = runtime(device);
    return owner != nullptr && owner->should_run();
}

static bool readable_stream_is_running(airspy_device* const device)
{
    const ReadableRuntime* const owner = runtime(device);
    return owner != nullptr && owner->is_running();
}

static void readable_stream_request_stop(airspy_device* const device)
{
    ReadableRuntime* const owner = runtime(device);
    if (owner != nullptr) {
        owner->request_stop();
    }
}

static void readable_stream_mark_stopped(airspy_device* const device)
{
    ReadableRuntime* const owner = runtime(device);
    if (owner != nullptr) {
        owner->mark_stopped();
    }
}

static int readable_create_io_threads(
    airspy_device* const device,
    const airspy_sample_block_cb_fn callback)
{
    ReadableRuntime* const owner = runtime(device);
    if (owner == nullptr || callback == nullptr) {
        return AIRSPY_ERROR_INVALID_PARAM;
    }
    if (!owner->begin_start()) {
        return AIRSPY_ERROR_BUSY;
    }

    device->callback = callback;
    int result = prepare_transfers(
        device,
        LIBUSB_ENDPOINT_IN | 1,
        airspy_libusb_transfer_callback);
    if (result != AIRSPY_SUCCESS) {
        return result;
    }

    pthread_attr_t attributes;
    if (pthread_attr_init(&attributes) != 0) {
        owner->drain_in_calling_thread();
        return AIRSPY_ERROR_THREAD;
    }
    (void)pthread_attr_setdetachstate(
        &attributes,
        PTHREAD_CREATE_JOINABLE);

    result = pthread_create(
        &device->consumer_thread,
        &attributes,
        consumer_threadproc,
        device);
    if (result != 0) {
        pthread_attr_destroy(&attributes);
        owner->drain_in_calling_thread();
        return AIRSPY_ERROR_THREAD;
    }
    owner->set_consumer_worker_running(true);

    result = pthread_create(
        &device->transfer_thread,
        &attributes,
        transfer_threadproc,
        device);
    pthread_attr_destroy(&attributes);
    if (result != 0) {
        owner->request_stop();
        pthread_mutex_lock(&device->consumer_mp);
        pthread_cond_signal(&device->consumer_cv);
        pthread_mutex_unlock(&device->consumer_mp);
        owner->drain_in_calling_thread();
        pthread_join(device->consumer_thread, nullptr);
        owner->set_consumer_worker_running(false);
        return AIRSPY_ERROR_THREAD;
    }
    owner->set_transfer_worker_running(true);
    return AIRSPY_SUCCESS;
}

static int readable_kill_io_threads(airspy_device* const device)
{
    ReadableRuntime* const owner = runtime(device);
    if (owner == nullptr) {
        return AIRSPY_ERROR_INVALID_PARAM;
    }
    std::lock_guard lifecycle_lock(owner->lifecycle_mutex());

    owner->request_stop();
    pthread_mutex_lock(&device->consumer_mp);
    pthread_cond_signal(&device->consumer_cv);
    pthread_mutex_unlock(&device->consumer_mp);

    const pthread_t self = pthread_self();
    if (owner->transfer_worker_running()
        && !pthread_equal(self, device->transfer_thread)) {
        const int join_result = pthread_join(device->transfer_thread, nullptr);
        if (join_result != 0) {
            return AIRSPY_ERROR_THREAD;
        }
        owner->set_transfer_worker_running(false);
    } else if (!owner->transfer_worker_running() && owner->pending() != 0) {
        owner->drain_in_calling_thread();
    }

    if (owner->consumer_worker_running()
        && !pthread_equal(self, device->consumer_thread)) {
        const int join_result = pthread_join(device->consumer_thread, nullptr);
        if (join_result != 0) {
            return AIRSPY_ERROR_THREAD;
        }
        owner->set_consumer_worker_running(false);
    }

    return owner->pending() == 0
        ? AIRSPY_SUCCESS
        : AIRSPY_ERROR_STREAMING_THREAD_ERR;
}

static int readable_start_rx(
    airspy_device* const device,
    const airspy_sample_block_cb_fn callback,
    void* const context)
{
    if (device == nullptr || callback == nullptr
        || device->cnv_f == nullptr || device->cnv_i == nullptr) {
        return AIRSPY_ERROR_INVALID_PARAM;
    }
    ReadableRuntime* const owner = runtime(device);
    if (owner == nullptr) {
        return AIRSPY_ERROR_INVALID_PARAM;
    }
    std::lock_guard lifecycle_lock(owner->lifecycle_mutex());

    iqconverter_float_reset(device->cnv_f);
    iqconverter_int16_reset(device->cnv_i);
    std::memset(
        device->dropped_buffers_queue,
        0,
        RAW_BUFFER_COUNT * sizeof(std::uint32_t));
    device->dropped_buffers = 0;

    int result =
        airspy_set_receiver_mode(device, RECEIVER_MODE_OFF);
    if (result != AIRSPY_SUCCESS) {
        return result;
    }
    (void)libusb_clear_halt(
        device->usb_device,
        LIBUSB_ENDPOINT_IN | 1);

    result = airspy_set_receiver_mode(device, RECEIVER_MODE_ARMED);
    if (result == AIRSPY_SUCCESS) {
        device->ctx = context;
        result = create_io_threads(device, callback);
        if (result != AIRSPY_SUCCESS) {
            (void)airspy_set_receiver_mode(device, RECEIVER_MODE_OFF);
            return result;
        }
        result = airspy_set_receiver_mode(device, RECEIVER_MODE_RX);
        if (result != AIRSPY_SUCCESS) {
            (void)airspy_set_receiver_mode(device, RECEIVER_MODE_OFF);
            (void)kill_io_threads(device);
        }
        return result;
    }

    result = airspy_set_receiver_mode(device, RECEIVER_MODE_RX);
    if (result != AIRSPY_SUCCESS) {
        return result;
    }
    device->ctx = context;
    result = create_io_threads(device, callback);
    if (result != AIRSPY_SUCCESS) {
        (void)airspy_set_receiver_mode(device, RECEIVER_MODE_OFF);
    }
    return result;
}

static int readable_stop_rx(airspy_device* const device)
{
    if (device == nullptr) {
        return AIRSPY_ERROR_INVALID_PARAM;
    }
    ReadableRuntime* const owner = runtime(device);
    if (owner == nullptr) {
        return AIRSPY_ERROR_INVALID_PARAM;
    }

    /*
     * This check deliberately precedes lifecycle serialization. If an
     * external finalizer already owns the lifecycle mutex and is joining the
     * consumer, the callback must be able to request stop and return instead
     * of waiting on the mutex that the joining thread holds.
     */
    if (owner->consumer_worker_running()
        && pthread_equal(pthread_self(), device->consumer_thread)) {
        readable_stream_request_stop(device);
        const int mode_result =
            airspy_set_receiver_mode(device, RECEIVER_MODE_OFF);
        pthread_mutex_lock(&device->consumer_mp);
        pthread_cond_signal(&device->consumer_cv);
        pthread_mutex_unlock(&device->consumer_mp);
        return mode_result;
    }

    std::lock_guard lifecycle_lock(owner->lifecycle_mutex());
    readable_stream_request_stop(device);
    const int mode_result =
        airspy_set_receiver_mode(device, RECEIVER_MODE_OFF);
    const int thread_result = kill_io_threads(device);
    return mode_result != AIRSPY_SUCCESS ? mode_result : thread_result;
}

static int readable_close(airspy_device* const device)
{
    if (device == nullptr) {
        return AIRSPY_SUCCESS;
    }
    ReadableRuntime* const owner = runtime(device);
    if (owner == nullptr) {
        return AIRSPY_ERROR_INVALID_PARAM;
    }
    std::unique_lock lifecycle_lock(owner->lifecycle_mutex());

    const int stop_result = readable_stop_rx(device);
    if (stop_result != AIRSPY_SUCCESS) {
        return stop_result;
    }
    if (!readable_runtime_prepare_free(device)) {
        return AIRSPY_ERROR_BUSY;
    }

    iqconverter_float_free(device->cnv_f);
    iqconverter_int16_free(device->cnv_i);
    device->cnv_f = nullptr;
    device->cnv_i = nullptr;

    const int free_result = free_transfers(device);
    if (free_result != AIRSPY_SUCCESS) {
        return free_result;
    }
    pthread_cond_destroy(&device->consumer_cv);
    pthread_mutex_destroy(&device->consumer_mp);
    airspy_open_exit(device);
    std::free(device->supported_samplerates);

    /*
     * The lifecycle mutex belongs to ReadableRuntime. Release it before
     * destroying the runtime; otherwise the lock destructor attempts to
     * unlock storage that has already been freed.
     */
    lifecycle_lock.unlock();
    readable_runtime_destroy(device);
    std::free(device);
    return AIRSPY_SUCCESS;
}
