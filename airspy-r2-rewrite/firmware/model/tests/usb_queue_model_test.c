#include "airspy_firmware/usb_queue_model.h"
#include "airspy_firmware/stream_status_model.h"
#include "airspy_firmware/transport_pump_model.h"

#include <assert.h>
#include <stddef.h>
#include <stdint.h>

enum {
    BUFFER_BYTES = 16384,
    SAMPLES_PER_BUFFER = 8192
};

static size_t make_ready(airspy_buffer_ring* buffers)
{
    size_t index = AIRSPY_USB_NO_INDEX;
    assert(airspy_buffer_acquire_for_dma(buffers, &index) == AIRSPY_BUFFER_OK);
    assert(airspy_buffer_complete_dma(
               buffers, index, BUFFER_BYTES, SAMPLES_PER_BUFFER)
        == AIRSPY_BUFFER_OK);
    assert(airspy_buffer_prepare_usb(buffers, index, BUFFER_BYTES)
        == AIRSPY_BUFFER_OK);
    return index;
}

static size_t find_ready(const airspy_buffer_ring* buffers)
{
    size_t selected = AIRSPY_USB_NO_INDEX;
    uint64_t selected_sequence = UINT64_MAX;
    for (size_t index = 0; index < buffers->slot_count; ++index) {
        if (buffers->slots[index].state == AIRSPY_BUFFER_READY_USB
            && buffers->slots[index].first_sample_sequence < selected_sequence) {
            selected = index;
            selected_sequence = buffers->slots[index].first_sample_sequence;
        }
    }
    return selected;
}

static void test_legacy_one_descriptor(void)
{
    const uintptr_t addresses[] = {0x20004000u, 0x20008000u};
    airspy_buffer_ring buffers;
    airspy_usb_queue queue;

    assert(airspy_buffer_ring_initialize(
               &buffers, addresses, 2, BUFFER_BYTES, 1)
        == AIRSPY_BUFFER_OK);
    assert(airspy_usb_queue_initialize(&queue, 1) == AIRSPY_USB_QUEUE_OK);

    const size_t first = make_ready(&buffers);
    const size_t second = make_ready(&buffers);
    assert(airspy_usb_queue_submit(&queue, &buffers, first)
        == AIRSPY_USB_QUEUE_OK);
    assert(airspy_usb_queue_prime(&queue, &buffers) == AIRSPY_USB_QUEUE_OK);
    assert(airspy_usb_queue_submit(&queue, &buffers, second)
        == AIRSPY_USB_QUEUE_WOULD_BLOCK);
    assert(buffers.slots[first].state == AIRSPY_BUFFER_USB_ACTIVE);
    assert(buffers.slots[second].state == AIRSPY_BUFFER_READY_USB);
    assert(airspy_usb_queue_validate(&queue, &buffers));

    size_t unavailable = AIRSPY_USB_NO_INDEX;
    assert(airspy_buffer_acquire_for_dma(&buffers, &unavailable)
        == AIRSPY_BUFFER_WOULD_BLOCK);
    assert(airspy_usb_queue_complete(&queue, &buffers) == AIRSPY_USB_QUEUE_OK);
    assert(airspy_usb_queue_submit(&queue, &buffers, second)
        == AIRSPY_USB_QUEUE_OK);
    assert(airspy_usb_queue_prime(&queue, &buffers) == AIRSPY_USB_QUEUE_OK);
    assert(airspy_usb_queue_complete(&queue, &buffers) == AIRSPY_USB_QUEUE_OK);

    assert(queue.high_watermark == 1);
    assert(queue.backpressure_count == 1);
    assert(queue.prime_count == 2);
    assert(queue.completion_count == 2);
    assert(queue.retired_samples == 2u * SAMPLES_PER_BUFFER);
    assert(airspy_usb_queue_validate(&queue, &buffers));

    airspy_one_stream_status status;
    assert(airspy_stream_status_snapshot(&buffers, &queue, 1, &status) == 0);
    assert(status.generation == 1);
    assert(status.captured_samples == 2u * SAMPLES_PER_BUFFER);
    assert(status.submitted_samples == 2u * SAMPLES_PER_BUFFER);
    assert(status.retired_samples == 2u * SAMPLES_PER_BUFFER);
    assert(status.queue_high_watermark == 1);
    assert(status.capture_buffer_count == 2);
    assert(status.descriptor_count == 1);
    assert((status.flags & AIRSPY_ONE_STREAM_CAPTURE_BACKPRESSURE) != 0);
}

static void test_nonblocking_transport_pump(void)
{
    const uintptr_t addresses[] = {0x20004000u, 0x20008000u};
    airspy_buffer_ring buffers;
    airspy_usb_queue queue;
    airspy_transport_pump_report report;

    assert(airspy_buffer_ring_initialize(
               &buffers, addresses, 2, BUFFER_BYTES, 2)
        == AIRSPY_BUFFER_OK);
    assert(airspy_usb_queue_initialize(&queue, 1) == AIRSPY_USB_QUEUE_OK);
    const size_t first = make_ready(&buffers);
    const size_t second = make_ready(&buffers);

    assert(airspy_transport_pump(&buffers, &queue, &report)
        == AIRSPY_USB_QUEUE_OK);
    assert(report.submitted_buffers == 1);
    assert(report.endpoint_primed);
    assert(report.waiting_for_descriptor);
    assert(buffers.slots[first].state == AIRSPY_BUFFER_USB_ACTIVE);
    assert(buffers.slots[second].state == AIRSPY_BUFFER_READY_USB);

    assert(airspy_transport_pump(&buffers, &queue, &report)
        == AIRSPY_USB_QUEUE_OK);
    assert(report.submitted_buffers == 0);
    assert(!report.endpoint_primed);
    assert(report.waiting_for_descriptor);

    assert(airspy_usb_queue_complete(&queue, &buffers) == AIRSPY_USB_QUEUE_OK);
    assert(airspy_transport_pump(&buffers, &queue, &report)
        == AIRSPY_USB_QUEUE_OK);
    assert(report.submitted_buffers == 1);
    assert(report.endpoint_primed);
    assert(!report.waiting_for_descriptor);
}

static void test_three_descriptor_chain(void)
{
    /*
     * The last two addresses are deliberately synthetic.  This test proves
     * ownership and queue behavior, not LPC43xx USB-DMA visibility.
     */
    const uintptr_t addresses[] = {
        0x20004000u, 0x20008000u, 0x21000000u, 0x21004000u};
    airspy_buffer_ring buffers;
    airspy_usb_queue queue;
    size_t slots[4];

    assert(airspy_buffer_ring_initialize(
               &buffers, addresses, 4, BUFFER_BYTES, 9)
        == AIRSPY_BUFFER_OK);
    assert(airspy_usb_queue_initialize(&queue, 3) == AIRSPY_USB_QUEUE_OK);
    for (size_t index = 0; index < 4; ++index) {
        slots[index] = make_ready(&buffers);
        assert(buffers.slots[slots[index]].first_sample_sequence
            == index * (uint64_t)SAMPLES_PER_BUFFER);
    }

    for (size_t index = 0; index < 3; ++index) {
        assert(airspy_usb_queue_submit(&queue, &buffers, slots[index])
            == AIRSPY_USB_QUEUE_OK);
    }
    assert(airspy_usb_queue_prime(&queue, &buffers) == AIRSPY_USB_QUEUE_OK);
    assert(airspy_usb_queue_submit(&queue, &buffers, slots[3])
        == AIRSPY_USB_QUEUE_WOULD_BLOCK);
    assert(queue.high_watermark == 3);
    assert(airspy_usb_queue_validate(&queue, &buffers));

    assert(airspy_usb_queue_complete(&queue, &buffers) == AIRSPY_USB_QUEUE_OK);
    assert(queue.active_index != AIRSPY_USB_NO_INDEX);
    assert(queue.prime_count == 1);
    assert(airspy_usb_queue_submit(&queue, &buffers, slots[3])
        == AIRSPY_USB_QUEUE_OK);
    assert(airspy_usb_queue_validate(&queue, &buffers));

    size_t recycled = AIRSPY_USB_NO_INDEX;
    assert(airspy_buffer_acquire_for_dma(&buffers, &recycled)
        == AIRSPY_BUFFER_OK);
    assert(recycled == slots[0]);
    assert(airspy_buffer_complete_dma(
               &buffers, recycled, BUFFER_BYTES, SAMPLES_PER_BUFFER)
        == AIRSPY_BUFFER_OK);
    assert(airspy_buffer_prepare_usb(&buffers, recycled, BUFFER_BYTES)
        == AIRSPY_BUFFER_OK);

    while (queue.active_index != AIRSPY_USB_NO_INDEX) {
        assert(airspy_usb_queue_complete(&queue, &buffers)
            == AIRSPY_USB_QUEUE_OK);
        assert(airspy_usb_queue_validate(&queue, &buffers));
    }
    assert(queue.completion_count == 4);
    assert(queue.retired_samples == 4u * SAMPLES_PER_BUFFER);
    assert(buffers.slots[recycled].state == AIRSPY_BUFFER_READY_USB);
}

static void test_cancel_releases_every_reference(void)
{
    const uintptr_t addresses[] = {
        0x20004000u, 0x20008000u, 0x21000000u};
    airspy_buffer_ring buffers;
    airspy_usb_queue queue;

    assert(airspy_buffer_ring_initialize(
               &buffers, addresses, 3, BUFFER_BYTES, 4)
        == AIRSPY_BUFFER_OK);
    assert(airspy_usb_queue_initialize(&queue, 3) == AIRSPY_USB_QUEUE_OK);
    for (size_t index = 0; index < 3; ++index) {
        const size_t buffer = make_ready(&buffers);
        assert(airspy_usb_queue_submit(&queue, &buffers, buffer)
            == AIRSPY_USB_QUEUE_OK);
    }
    assert(airspy_usb_queue_prime(&queue, &buffers) == AIRSPY_USB_QUEUE_OK);
    assert(airspy_usb_queue_cancel_all(&queue, &buffers)
        == AIRSPY_USB_QUEUE_OK);
    for (size_t index = 0; index < buffers.slot_count; ++index) {
        assert(buffers.slots[index].state == AIRSPY_BUFFER_FREE);
    }
    assert(queue.in_flight_count == 0);
    assert(queue.cancellation_count == 1);
    assert(airspy_usb_queue_validate(&queue, &buffers));
}

static void test_corrupt_chain_is_rejected_before_retirement(void)
{
    const uintptr_t addresses[] = {0x20004000u, 0x20008000u};
    airspy_buffer_ring buffers;
    airspy_usb_queue queue;

    assert(airspy_buffer_ring_initialize(
               &buffers, addresses, 2, BUFFER_BYTES, 5)
        == AIRSPY_BUFFER_OK);
    assert(airspy_usb_queue_initialize(&queue, 2) == AIRSPY_USB_QUEUE_OK);
    const size_t first = make_ready(&buffers);
    const size_t second = make_ready(&buffers);
    assert(airspy_usb_queue_submit(&queue, &buffers, first)
        == AIRSPY_USB_QUEUE_OK);
    assert(airspy_usb_queue_submit(&queue, &buffers, second)
        == AIRSPY_USB_QUEUE_OK);
    assert(airspy_usb_queue_prime(&queue, &buffers) == AIRSPY_USB_QUEUE_OK);

    queue.descriptors[queue.tail_index].next_index = queue.head_index;
    assert(!airspy_usb_queue_validate(&queue, &buffers));
    assert(airspy_usb_queue_complete(&queue, &buffers)
        == AIRSPY_USB_QUEUE_ILLEGAL_STATE);
    assert(buffers.slots[first].state == AIRSPY_BUFFER_USB_ACTIVE);
    assert(buffers.slots[second].state == AIRSPY_BUFFER_USB_QUEUED);
    assert(queue.in_flight_count == 2);
}

static void test_sustained_pipeline(void)
{
    const uintptr_t addresses[] = {
        0x20004000u, 0x20008000u, 0x21000000u, 0x21004000u};
    airspy_buffer_ring buffers;
    airspy_usb_queue queue;

    assert(airspy_buffer_ring_initialize(
               &buffers, addresses, 4, BUFFER_BYTES, 42)
        == AIRSPY_BUFFER_OK);
    assert(airspy_usb_queue_initialize(&queue, 3) == AIRSPY_USB_QUEUE_OK);

    for (size_t iteration = 0; iteration < 50000; ++iteration) {
        size_t acquired = AIRSPY_USB_NO_INDEX;
        if (airspy_buffer_acquire_for_dma(&buffers, &acquired)
            == AIRSPY_BUFFER_OK) {
            assert(airspy_buffer_complete_dma(
                       &buffers, acquired, BUFFER_BYTES, SAMPLES_PER_BUFFER)
                == AIRSPY_BUFFER_OK);
            assert(airspy_buffer_prepare_usb(
                       &buffers, acquired, BUFFER_BYTES)
                == AIRSPY_BUFFER_OK);
        }

        for (;;) {
            const size_t ready = find_ready(&buffers);
            if (ready == AIRSPY_USB_NO_INDEX) {
                break;
            }
            const airspy_usb_queue_result result =
                airspy_usb_queue_submit(&queue, &buffers, ready);
            if (result == AIRSPY_USB_QUEUE_WOULD_BLOCK) {
                break;
            }
            assert(result == AIRSPY_USB_QUEUE_OK);
        }
        if (queue.active_index == AIRSPY_USB_NO_INDEX
            && queue.head_index != AIRSPY_USB_NO_INDEX) {
            assert(airspy_usb_queue_prime(&queue, &buffers)
                == AIRSPY_USB_QUEUE_OK);
        }
        if (queue.active_index != AIRSPY_USB_NO_INDEX
            && iteration % 3 != 0) {
            assert(airspy_usb_queue_complete(&queue, &buffers)
                == AIRSPY_USB_QUEUE_OK);
        }
        assert(airspy_usb_queue_validate(&queue, &buffers));
    }

    while (queue.active_index != AIRSPY_USB_NO_INDEX) {
        assert(airspy_usb_queue_complete(&queue, &buffers)
            == AIRSPY_USB_QUEUE_OK);
        assert(airspy_usb_queue_validate(&queue, &buffers));
    }
    assert(queue.completion_count > 30000);
    assert(queue.high_watermark == 3);
    assert(queue.illegal_state_count == 0);
}

int main(void)
{
    test_legacy_one_descriptor();
    test_nonblocking_transport_pump();
    test_three_descriptor_chain();
    test_cancel_releases_every_reference();
    test_corrupt_chain_is_rejected_before_retirement();
    test_sustained_pipeline();
    return 0;
}
