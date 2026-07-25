#include "airspy_firmware/usb_queue_model.h"

#include <string.h>

static void clear_descriptor(airspy_usb_dtd* descriptor)
{
    memset(descriptor, 0, sizeof(*descriptor));
    descriptor->next_index = AIRSPY_USB_NO_INDEX;
    descriptor->buffer_index = AIRSPY_USB_NO_INDEX;
    descriptor->state = AIRSPY_USB_DTD_FREE;
}

static size_t acquire_descriptor(airspy_usb_queue* queue)
{
    for (size_t offset = 0; offset < queue->descriptor_count; ++offset) {
        const size_t index =
            (queue->allocation_cursor + offset) % queue->descriptor_count;
        if (queue->descriptors[index].state == AIRSPY_USB_DTD_FREE) {
            queue->allocation_cursor = (index + 1) % queue->descriptor_count;
            return index;
        }
    }
    return AIRSPY_USB_NO_INDEX;
}

airspy_usb_queue_result airspy_usb_queue_initialize(
    airspy_usb_queue* queue,
    size_t descriptor_count)
{
    if (queue == NULL || descriptor_count == 0
        || descriptor_count > AIRSPY_USB_MAX_MODEL_DTDS) {
        return AIRSPY_USB_QUEUE_BAD_ARGUMENT;
    }
    memset(queue, 0, sizeof(*queue));
    queue->descriptor_count = descriptor_count;
    queue->head_index = AIRSPY_USB_NO_INDEX;
    queue->tail_index = AIRSPY_USB_NO_INDEX;
    queue->active_index = AIRSPY_USB_NO_INDEX;
    for (size_t index = 0; index < descriptor_count; ++index) {
        clear_descriptor(&queue->descriptors[index]);
    }
    return AIRSPY_USB_QUEUE_OK;
}

airspy_usb_queue_result airspy_usb_queue_submit(
    airspy_usb_queue* queue,
    airspy_buffer_ring* buffers,
    size_t buffer_index)
{
    if (queue == NULL || buffers == NULL || buffer_index >= buffers->slot_count) {
        return AIRSPY_USB_QUEUE_BAD_ARGUMENT;
    }
    if (!airspy_usb_queue_validate(queue, buffers)) {
        ++queue->illegal_state_count;
        return AIRSPY_USB_QUEUE_ILLEGAL_STATE;
    }
    airspy_buffer_slot* const buffer = &buffers->slots[buffer_index];
    if (buffer->generation != buffers->generation) {
        return AIRSPY_USB_QUEUE_STALE_BUFFER;
    }
    if (buffer->state != AIRSPY_BUFFER_READY_USB || buffer->valid_bytes == 0) {
        ++queue->illegal_state_count;
        return AIRSPY_USB_QUEUE_ILLEGAL_STATE;
    }

    const size_t descriptor_index = acquire_descriptor(queue);
    if (descriptor_index == AIRSPY_USB_NO_INDEX) {
        ++queue->backpressure_count;
        return AIRSPY_USB_QUEUE_WOULD_BLOCK;
    }
    if (airspy_buffer_queue_usb(buffers, buffer_index) != AIRSPY_BUFFER_OK) {
        ++queue->illegal_state_count;
        return AIRSPY_USB_QUEUE_ILLEGAL_STATE;
    }

    airspy_usb_dtd* const descriptor = &queue->descriptors[descriptor_index];
    descriptor->next_index = AIRSPY_USB_NO_INDEX;
    descriptor->buffer_index = buffer_index;
    descriptor->buffer_generation = buffer->generation;
    descriptor->transfer_bytes = buffer->valid_bytes;
    descriptor->first_sample_sequence = buffer->first_sample_sequence;
    descriptor->state = AIRSPY_USB_DTD_LINKED;

    if (queue->tail_index == AIRSPY_USB_NO_INDEX) {
        queue->head_index = descriptor_index;
    } else {
        queue->descriptors[queue->tail_index].next_index = descriptor_index;
        ++queue->append_count;
    }
    queue->tail_index = descriptor_index;
    ++queue->in_flight_count;
    queue->submitted_samples += buffer->sample_count;
    if (queue->in_flight_count > queue->high_watermark) {
        queue->high_watermark = queue->in_flight_count;
    }
    return AIRSPY_USB_QUEUE_OK;
}

airspy_usb_queue_result airspy_usb_queue_prime(
    airspy_usb_queue* queue,
    airspy_buffer_ring* buffers)
{
    if (queue == NULL || buffers == NULL) {
        return AIRSPY_USB_QUEUE_BAD_ARGUMENT;
    }
    if (!airspy_usb_queue_validate(queue, buffers)) {
        ++queue->illegal_state_count;
        return AIRSPY_USB_QUEUE_ILLEGAL_STATE;
    }
    if (queue->active_index != AIRSPY_USB_NO_INDEX) {
        return AIRSPY_USB_QUEUE_OK;
    }
    if (queue->head_index == AIRSPY_USB_NO_INDEX) {
        return AIRSPY_USB_QUEUE_WOULD_BLOCK;
    }

    airspy_usb_dtd* const descriptor = &queue->descriptors[queue->head_index];
    if (descriptor->state != AIRSPY_USB_DTD_LINKED
        || descriptor->buffer_index >= buffers->slot_count
        || descriptor->buffer_generation != buffers->generation
        || airspy_buffer_activate_usb(
               buffers, descriptor->buffer_index) != AIRSPY_BUFFER_OK) {
        ++queue->illegal_state_count;
        return AIRSPY_USB_QUEUE_ILLEGAL_STATE;
    }
    descriptor->state = AIRSPY_USB_DTD_ACTIVE;
    queue->active_index = queue->head_index;
    ++queue->prime_count;
    return AIRSPY_USB_QUEUE_OK;
}

airspy_usb_queue_result airspy_usb_queue_complete(
    airspy_usb_queue* queue,
    airspy_buffer_ring* buffers)
{
    if (queue == NULL || buffers == NULL) {
        return AIRSPY_USB_QUEUE_BAD_ARGUMENT;
    }
    if (!airspy_usb_queue_validate(queue, buffers)) {
        ++queue->illegal_state_count;
        return AIRSPY_USB_QUEUE_ILLEGAL_STATE;
    }
    if (queue->active_index == AIRSPY_USB_NO_INDEX) {
        ++queue->illegal_state_count;
        return AIRSPY_USB_QUEUE_ILLEGAL_STATE;
    }

    const size_t completed_index = queue->active_index;
    airspy_usb_dtd* const completed = &queue->descriptors[completed_index];
    if (completed->state != AIRSPY_USB_DTD_ACTIVE
        || completed->buffer_index >= buffers->slot_count
        || completed->buffer_generation != buffers->generation) {
        ++queue->illegal_state_count;
        return AIRSPY_USB_QUEUE_ILLEGAL_STATE;
    }
    const size_t next_index = completed->next_index;
    const size_t buffer_index = completed->buffer_index;
    const uint32_t sample_count = buffers->slots[buffer_index].sample_count;
    if (airspy_buffer_retire_usb(buffers, buffer_index) != AIRSPY_BUFFER_OK) {
        ++queue->illegal_state_count;
        return AIRSPY_USB_QUEUE_ILLEGAL_STATE;
    }
    clear_descriptor(completed);
    --queue->in_flight_count;
    ++queue->completion_count;
    queue->retired_samples += sample_count;
    queue->head_index = next_index;
    queue->active_index = AIRSPY_USB_NO_INDEX;

    if (next_index == AIRSPY_USB_NO_INDEX) {
        queue->tail_index = AIRSPY_USB_NO_INDEX;
        return AIRSPY_USB_QUEUE_OK;
    }

    airspy_usb_dtd* const next = &queue->descriptors[next_index];
    if (next->state != AIRSPY_USB_DTD_LINKED
        || next->buffer_index >= buffers->slot_count
        || airspy_buffer_activate_usb(
               buffers, next->buffer_index) != AIRSPY_BUFFER_OK) {
        ++queue->illegal_state_count;
        return AIRSPY_USB_QUEUE_ILLEGAL_STATE;
    }
    next->state = AIRSPY_USB_DTD_ACTIVE;
    queue->active_index = next_index;
    return AIRSPY_USB_QUEUE_OK;
}

airspy_usb_queue_result airspy_usb_queue_cancel_all(
    airspy_usb_queue* queue,
    airspy_buffer_ring* buffers)
{
    if (queue == NULL || buffers == NULL) {
        return AIRSPY_USB_QUEUE_BAD_ARGUMENT;
    }
    if (!airspy_usb_queue_validate(queue, buffers)) {
        ++queue->illegal_state_count;
        return AIRSPY_USB_QUEUE_ILLEGAL_STATE;
    }
    size_t index = queue->head_index;
    size_t visited = 0;
    while (index != AIRSPY_USB_NO_INDEX) {
        if (index >= queue->descriptor_count || visited++ >= queue->descriptor_count) {
            ++queue->illegal_state_count;
            return AIRSPY_USB_QUEUE_ILLEGAL_STATE;
        }
        airspy_usb_dtd* const descriptor = &queue->descriptors[index];
        const size_t next = descriptor->next_index;
        if (descriptor->buffer_index >= buffers->slot_count
            || airspy_buffer_cancel_usb(
                   buffers, descriptor->buffer_index) != AIRSPY_BUFFER_OK) {
            ++queue->illegal_state_count;
            return AIRSPY_USB_QUEUE_ILLEGAL_STATE;
        }
        clear_descriptor(descriptor);
        index = next;
    }
    queue->head_index = AIRSPY_USB_NO_INDEX;
    queue->tail_index = AIRSPY_USB_NO_INDEX;
    queue->active_index = AIRSPY_USB_NO_INDEX;
    queue->in_flight_count = 0;
    ++queue->cancellation_count;
    return AIRSPY_USB_QUEUE_OK;
}

int airspy_usb_queue_validate(
    const airspy_usb_queue* queue,
    const airspy_buffer_ring* buffers)
{
    if (queue == NULL || buffers == NULL || buffers->slot_count == 0
        || buffers->slot_count > AIRSPY_FIRMWARE_MAX_MODEL_BUFFERS
        || queue->descriptor_count == 0
        || queue->descriptor_count > AIRSPY_USB_MAX_MODEL_DTDS) {
        return 0;
    }

    uint8_t descriptor_seen[AIRSPY_USB_MAX_MODEL_DTDS] = {0};
    uint8_t buffer_seen[AIRSPY_FIRMWARE_MAX_MODEL_BUFFERS] = {0};
    size_t index = queue->head_index;
    size_t previous = AIRSPY_USB_NO_INDEX;
    size_t walked = 0;
    while (index != AIRSPY_USB_NO_INDEX) {
        if (index >= queue->descriptor_count || descriptor_seen[index]
            || walked >= queue->descriptor_count) {
            return 0;
        }
        descriptor_seen[index] = 1;
        const airspy_usb_dtd* const descriptor = &queue->descriptors[index];
        if (descriptor->buffer_index >= buffers->slot_count
            || buffer_seen[descriptor->buffer_index]
            || descriptor->buffer_generation != buffers->generation
            || descriptor->transfer_bytes == 0) {
            return 0;
        }
        buffer_seen[descriptor->buffer_index] = 1;
        if (index == queue->active_index) {
            if (index != queue->head_index
                || descriptor->state != AIRSPY_USB_DTD_ACTIVE
                || buffers->slots[descriptor->buffer_index].state
                    != AIRSPY_BUFFER_USB_ACTIVE) {
                return 0;
            }
        } else if (descriptor->state != AIRSPY_USB_DTD_LINKED
                   || buffers->slots[descriptor->buffer_index].state
                       != AIRSPY_BUFFER_USB_QUEUED) {
            return 0;
        }
        previous = index;
        index = descriptor->next_index;
        ++walked;
    }
    if (walked != queue->in_flight_count) return 0;
    if (!walked) {
        if (queue->head_index != AIRSPY_USB_NO_INDEX
            || queue->tail_index != AIRSPY_USB_NO_INDEX
            || queue->active_index != AIRSPY_USB_NO_INDEX) {
            return 0;
        }
    } else if (previous != queue->tail_index) {
        return 0;
    }

    for (size_t descriptor = 0; descriptor < queue->descriptor_count;
         ++descriptor) {
        if (!descriptor_seen[descriptor]
            && queue->descriptors[descriptor].state != AIRSPY_USB_DTD_FREE) {
            return 0;
        }
    }
    for (size_t buffer = 0; buffer < buffers->slot_count; ++buffer) {
        const airspy_buffer_state state = buffers->slots[buffer].state;
        const int usb_owned =
            state == AIRSPY_BUFFER_USB_QUEUED
            || state == AIRSPY_BUFFER_USB_ACTIVE;
        if (usb_owned != (buffer_seen[buffer] != 0)) {
            return 0;
        }
    }
    return 1;
}
