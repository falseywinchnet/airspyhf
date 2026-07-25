#ifndef AIRSPY_FIRMWARE_BUFFER_MODEL_H
#define AIRSPY_FIRMWARE_BUFFER_MODEL_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AIRSPY_FIRMWARE_MAX_MODEL_BUFFERS 8

typedef enum airspy_buffer_state {
    AIRSPY_BUFFER_FREE = 0,
    AIRSPY_BUFFER_DMA_FILLING,
    AIRSPY_BUFFER_READY_RAW,
    AIRSPY_BUFFER_PROCESSING,
    AIRSPY_BUFFER_READY_USB,
    AIRSPY_BUFFER_USB_QUEUED,
    AIRSPY_BUFFER_USB_ACTIVE
} airspy_buffer_state;

typedef enum airspy_buffer_result {
    AIRSPY_BUFFER_OK = 0,
    AIRSPY_BUFFER_WOULD_BLOCK = 1,
    AIRSPY_BUFFER_BAD_ARGUMENT = -1,
    AIRSPY_BUFFER_ILLEGAL_TRANSITION = -2,
    AIRSPY_BUFFER_STALE_GENERATION = -3
} airspy_buffer_result;

typedef struct airspy_buffer_slot {
    uintptr_t address;
    uint32_t capacity_bytes;
    uint32_t valid_bytes;
    uint32_t sample_count;
    uint32_t generation;
    uint64_t first_sample_sequence;
    airspy_buffer_state state;
} airspy_buffer_slot;

typedef struct airspy_buffer_ring {
    airspy_buffer_slot slots[AIRSPY_FIRMWARE_MAX_MODEL_BUFFERS];
    size_t slot_count;
    size_t dma_search_cursor;
    uint32_t generation;
    uint64_t next_sample_sequence;
    uint64_t dropped_samples;
    uint32_t dma_backpressure_count;
    uint32_t illegal_transition_count;
} airspy_buffer_ring;

airspy_buffer_result airspy_buffer_ring_initialize(
    airspy_buffer_ring* ring,
    const uintptr_t* addresses,
    size_t slot_count,
    uint32_t bytes_per_slot,
    uint32_t generation);

airspy_buffer_result airspy_buffer_acquire_for_dma(
    airspy_buffer_ring* ring,
    size_t* slot_index);

airspy_buffer_result airspy_buffer_complete_dma(
    airspy_buffer_ring* ring,
    size_t slot_index,
    uint32_t valid_bytes,
    uint32_t sample_count);

airspy_buffer_result airspy_buffer_prepare_usb(
    airspy_buffer_ring* ring,
    size_t slot_index,
    uint32_t valid_bytes);

airspy_buffer_result airspy_buffer_queue_usb(
    airspy_buffer_ring* ring,
    size_t slot_index);

airspy_buffer_result airspy_buffer_activate_usb(
    airspy_buffer_ring* ring,
    size_t slot_index);

airspy_buffer_result airspy_buffer_retire_usb(
    airspy_buffer_ring* ring,
    size_t slot_index);

airspy_buffer_result airspy_buffer_cancel_usb(
    airspy_buffer_ring* ring,
    size_t slot_index);

airspy_buffer_result airspy_buffer_record_drop(
    airspy_buffer_ring* ring,
    uint32_t sample_count);

#ifdef __cplusplus
}
#endif

#endif
