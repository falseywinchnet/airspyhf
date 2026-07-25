#include "airspy_firmware/buffer_model.h"

#include <assert.h>

int main(void)
{
    const uintptr_t addresses[] = {0x20004000u, 0x20008000u};
    airspy_buffer_ring ring;
    size_t first;
    size_t second;
    size_t unavailable;

    assert(airspy_buffer_ring_initialize(
               &ring, addresses, 2, 16384, 7)
        == AIRSPY_BUFFER_OK);
    assert(airspy_buffer_acquire_for_dma(&ring, &first) == AIRSPY_BUFFER_OK);
    assert(airspy_buffer_acquire_for_dma(&ring, &second) == AIRSPY_BUFFER_OK);
    assert(first != second);
    assert(airspy_buffer_acquire_for_dma(&ring, &unavailable)
        == AIRSPY_BUFFER_WOULD_BLOCK);
    assert(ring.dma_backpressure_count == 1);

    assert(airspy_buffer_complete_dma(&ring, first, 16384, 8192)
        == AIRSPY_BUFFER_OK);
    assert(ring.slots[first].first_sample_sequence == 0);
    assert(airspy_buffer_prepare_usb(&ring, first, 16384) == AIRSPY_BUFFER_OK);
    assert(airspy_buffer_queue_usb(&ring, first) == AIRSPY_BUFFER_OK);
    assert(airspy_buffer_activate_usb(&ring, first) == AIRSPY_BUFFER_OK);
    assert(airspy_buffer_retire_usb(&ring, first) == AIRSPY_BUFFER_OK);

    assert(airspy_buffer_queue_usb(&ring, first)
        == AIRSPY_BUFFER_ILLEGAL_TRANSITION);
    assert(ring.illegal_transition_count == 1);
    assert(airspy_buffer_acquire_for_dma(&ring, &unavailable)
        == AIRSPY_BUFFER_OK);
    assert(unavailable == first);
    assert(ring.slots[first].first_sample_sequence == 8192);
    assert(airspy_buffer_record_drop(&ring, 4096) == AIRSPY_BUFFER_OK);
    assert(ring.dropped_samples == 4096);
    assert(ring.next_sample_sequence == 12288);
    return 0;
}
