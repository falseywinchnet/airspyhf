#include "airspy_firmware/buffer_model.h"

#include <string.h>

static airspy_buffer_result transition(
    airspy_buffer_ring* ring,
    size_t slot_index,
    airspy_buffer_state expected,
    airspy_buffer_state next)
{
    if (ring == NULL || slot_index >= ring->slot_count) {
        return AIRSPY_BUFFER_BAD_ARGUMENT;
    }
    airspy_buffer_slot* const slot = &ring->slots[slot_index];
    if (slot->generation != ring->generation) {
        return AIRSPY_BUFFER_STALE_GENERATION;
    }
    if (slot->state != expected) {
        ++ring->illegal_transition_count;
        return AIRSPY_BUFFER_ILLEGAL_TRANSITION;
    }
    slot->state = next;
    return AIRSPY_BUFFER_OK;
}

airspy_buffer_result airspy_buffer_ring_initialize(
    airspy_buffer_ring* ring,
    const uintptr_t* addresses,
    size_t slot_count,
    uint32_t bytes_per_slot,
    uint32_t generation)
{
    if (ring == NULL || addresses == NULL || slot_count == 0
        || slot_count > AIRSPY_FIRMWARE_MAX_MODEL_BUFFERS
        || bytes_per_slot == 0) {
        return AIRSPY_BUFFER_BAD_ARGUMENT;
    }

    memset(ring, 0, sizeof(*ring));
    ring->slot_count = slot_count;
    ring->generation = generation;
    for (size_t index = 0; index < slot_count; ++index) {
        ring->slots[index].address = addresses[index];
        ring->slots[index].capacity_bytes = bytes_per_slot;
        ring->slots[index].generation = generation;
        ring->slots[index].state = AIRSPY_BUFFER_FREE;
    }
    return AIRSPY_BUFFER_OK;
}

airspy_buffer_result airspy_buffer_acquire_for_dma(
    airspy_buffer_ring* ring,
    size_t* slot_index)
{
    if (ring == NULL || slot_index == NULL) {
        return AIRSPY_BUFFER_BAD_ARGUMENT;
    }
    for (size_t offset = 0; offset < ring->slot_count; ++offset) {
        const size_t index = (ring->dma_search_cursor + offset) % ring->slot_count;
        airspy_buffer_slot* const slot = &ring->slots[index];
        if (slot->state == AIRSPY_BUFFER_FREE) {
            slot->valid_bytes = 0;
            slot->sample_count = 0;
            slot->first_sample_sequence = ring->next_sample_sequence;
            slot->state = AIRSPY_BUFFER_DMA_FILLING;
            ring->dma_search_cursor = (index + 1) % ring->slot_count;
            *slot_index = index;
            return AIRSPY_BUFFER_OK;
        }
    }
    ++ring->dma_backpressure_count;
    return AIRSPY_BUFFER_WOULD_BLOCK;
}

airspy_buffer_result airspy_buffer_complete_dma(
    airspy_buffer_ring* ring,
    size_t slot_index,
    uint32_t valid_bytes,
    uint32_t sample_count)
{
    if (ring == NULL || slot_index >= ring->slot_count
        || valid_bytes > ring->slots[slot_index].capacity_bytes) {
        return AIRSPY_BUFFER_BAD_ARGUMENT;
    }
    const airspy_buffer_result result = transition(
        ring, slot_index, AIRSPY_BUFFER_DMA_FILLING, AIRSPY_BUFFER_READY_RAW);
    if (result != AIRSPY_BUFFER_OK) {
        return result;
    }
    ring->slots[slot_index].valid_bytes = valid_bytes;
    ring->slots[slot_index].sample_count = sample_count;
    ring->next_sample_sequence += sample_count;
    return AIRSPY_BUFFER_OK;
}

airspy_buffer_result airspy_buffer_prepare_usb(
    airspy_buffer_ring* ring,
    size_t slot_index,
    uint32_t valid_bytes)
{
    if (ring == NULL || slot_index >= ring->slot_count
        || valid_bytes > ring->slots[slot_index].capacity_bytes) {
        return AIRSPY_BUFFER_BAD_ARGUMENT;
    }
    const airspy_buffer_result result = transition(
        ring, slot_index, AIRSPY_BUFFER_READY_RAW, AIRSPY_BUFFER_READY_USB);
    if (result == AIRSPY_BUFFER_OK) {
        ring->slots[slot_index].valid_bytes = valid_bytes;
    }
    return result;
}

airspy_buffer_result airspy_buffer_queue_usb(
    airspy_buffer_ring* ring,
    size_t slot_index)
{
    return transition(
        ring, slot_index, AIRSPY_BUFFER_READY_USB, AIRSPY_BUFFER_USB_QUEUED);
}

airspy_buffer_result airspy_buffer_activate_usb(
    airspy_buffer_ring* ring,
    size_t slot_index)
{
    return transition(
        ring, slot_index, AIRSPY_BUFFER_USB_QUEUED, AIRSPY_BUFFER_USB_ACTIVE);
}

airspy_buffer_result airspy_buffer_retire_usb(
    airspy_buffer_ring* ring,
    size_t slot_index)
{
    return transition(
        ring, slot_index, AIRSPY_BUFFER_USB_ACTIVE, AIRSPY_BUFFER_FREE);
}

airspy_buffer_result airspy_buffer_cancel_usb(
    airspy_buffer_ring* ring,
    size_t slot_index)
{
    if (ring == NULL || slot_index >= ring->slot_count) {
        return AIRSPY_BUFFER_BAD_ARGUMENT;
    }
    const airspy_buffer_state state = ring->slots[slot_index].state;
    if (state != AIRSPY_BUFFER_USB_QUEUED
        && state != AIRSPY_BUFFER_USB_ACTIVE) {
        ++ring->illegal_transition_count;
        return AIRSPY_BUFFER_ILLEGAL_TRANSITION;
    }
    if (ring->slots[slot_index].generation != ring->generation) {
        return AIRSPY_BUFFER_STALE_GENERATION;
    }
    ring->slots[slot_index].state = AIRSPY_BUFFER_FREE;
    return AIRSPY_BUFFER_OK;
}

airspy_buffer_result airspy_buffer_record_drop(
    airspy_buffer_ring* ring,
    uint32_t sample_count)
{
    if (ring == NULL || sample_count == 0) {
        return AIRSPY_BUFFER_BAD_ARGUMENT;
    }
    ring->dropped_samples += sample_count;
    ring->next_sample_sequence += sample_count;
    return AIRSPY_BUFFER_OK;
}
