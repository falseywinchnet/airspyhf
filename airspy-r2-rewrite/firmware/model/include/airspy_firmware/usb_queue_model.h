#ifndef AIRSPY_FIRMWARE_USB_QUEUE_MODEL_H
#define AIRSPY_FIRMWARE_USB_QUEUE_MODEL_H

#include "airspy_firmware/buffer_model.h"

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AIRSPY_USB_MAX_MODEL_DTDS 8
#define AIRSPY_USB_NO_INDEX ((size_t)-1)

typedef enum airspy_usb_dtd_state {
    AIRSPY_USB_DTD_FREE = 0,
    AIRSPY_USB_DTD_LINKED,
    AIRSPY_USB_DTD_ACTIVE
} airspy_usb_dtd_state;

typedef enum airspy_usb_queue_result {
    AIRSPY_USB_QUEUE_OK = 0,
    AIRSPY_USB_QUEUE_WOULD_BLOCK = 1,
    AIRSPY_USB_QUEUE_BAD_ARGUMENT = -1,
    AIRSPY_USB_QUEUE_ILLEGAL_STATE = -2,
    AIRSPY_USB_QUEUE_STALE_BUFFER = -3
} airspy_usb_queue_result;

typedef struct airspy_usb_dtd {
    size_t next_index;
    size_t buffer_index;
    uint32_t buffer_generation;
    uint32_t transfer_bytes;
    uint64_t first_sample_sequence;
    airspy_usb_dtd_state state;
} airspy_usb_dtd;

typedef struct airspy_usb_queue {
    airspy_usb_dtd descriptors[AIRSPY_USB_MAX_MODEL_DTDS];
    size_t descriptor_count;
    size_t allocation_cursor;
    size_t head_index;
    size_t tail_index;
    size_t active_index;
    size_t in_flight_count;
    size_t high_watermark;
    uint32_t prime_count;
    uint32_t append_count;
    uint32_t completion_count;
    uint32_t cancellation_count;
    uint32_t backpressure_count;
    uint32_t illegal_state_count;
    uint64_t submitted_samples;
    uint64_t retired_samples;
} airspy_usb_queue;

airspy_usb_queue_result airspy_usb_queue_initialize(
    airspy_usb_queue* queue,
    size_t descriptor_count);

airspy_usb_queue_result airspy_usb_queue_submit(
    airspy_usb_queue* queue,
    airspy_buffer_ring* buffers,
    size_t buffer_index);

airspy_usb_queue_result airspy_usb_queue_prime(
    airspy_usb_queue* queue,
    airspy_buffer_ring* buffers);

airspy_usb_queue_result airspy_usb_queue_complete(
    airspy_usb_queue* queue,
    airspy_buffer_ring* buffers);

airspy_usb_queue_result airspy_usb_queue_cancel_all(
    airspy_usb_queue* queue,
    airspy_buffer_ring* buffers);

int airspy_usb_queue_validate(
    const airspy_usb_queue* queue,
    const airspy_buffer_ring* buffers);

#ifdef __cplusplus
}
#endif

#endif
