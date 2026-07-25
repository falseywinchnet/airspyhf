#include "airspy_firmware/transport_pump_model.h"

#include <stdint.h>
#include <string.h>

static size_t oldest_ready_buffer(const airspy_buffer_ring* buffers)
{
    size_t selected = AIRSPY_USB_NO_INDEX;
    uint64_t selected_sequence = UINT64_MAX;
    for (size_t index = 0; index < buffers->slot_count; ++index) {
        const airspy_buffer_slot* const slot = &buffers->slots[index];
        if (slot->state == AIRSPY_BUFFER_READY_USB
            && slot->first_sample_sequence < selected_sequence) {
            selected = index;
            selected_sequence = slot->first_sample_sequence;
        }
    }
    return selected;
}

airspy_usb_queue_result airspy_transport_pump(
    airspy_buffer_ring* buffers,
    airspy_usb_queue* queue,
    airspy_transport_pump_report* report)
{
    if (buffers == NULL || queue == NULL || report == NULL) {
        return AIRSPY_USB_QUEUE_BAD_ARGUMENT;
    }
    memset(report, 0, sizeof(*report));
    if (!airspy_usb_queue_validate(queue, buffers)) {
        ++queue->illegal_state_count;
        return AIRSPY_USB_QUEUE_ILLEGAL_STATE;
    }

    for (size_t attempt = 0; attempt < buffers->slot_count; ++attempt) {
        const size_t ready = oldest_ready_buffer(buffers);
        if (ready == AIRSPY_USB_NO_INDEX) {
            break;
        }
        const airspy_usb_queue_result result =
            airspy_usb_queue_submit(queue, buffers, ready);
        if (result == AIRSPY_USB_QUEUE_WOULD_BLOCK) {
            report->waiting_for_descriptor = 1;
            break;
        }
        if (result != AIRSPY_USB_QUEUE_OK) {
            return result;
        }
        ++report->submitted_buffers;
    }

    if (queue->active_index == AIRSPY_USB_NO_INDEX
        && queue->head_index != AIRSPY_USB_NO_INDEX) {
        const airspy_usb_queue_result result =
            airspy_usb_queue_prime(queue, buffers);
        if (result != AIRSPY_USB_QUEUE_OK) {
            return result;
        }
        report->endpoint_primed = 1;
    }
    return AIRSPY_USB_QUEUE_OK;
}
