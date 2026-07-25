#include "airspy_firmware/stream_status_model.h"

#include <limits.h>
#include <string.h>

int airspy_stream_status_snapshot(
    const airspy_buffer_ring* buffers,
    const airspy_usb_queue* queue,
    int stream_running,
    airspy_one_stream_status* status)
{
    if (buffers == NULL || queue == NULL || status == NULL
        || buffers->slot_count > UINT8_MAX
        || queue->descriptor_count > UINT8_MAX
        || queue->high_watermark > UINT16_MAX) {
        return -1;
    }

    memset(status, 0, sizeof(*status));
    status->generation = buffers->generation;
    if (stream_running) {
        status->flags |= AIRSPY_ONE_STREAM_RUNNING;
    }
    if (buffers->dropped_samples != 0) {
        status->flags |= AIRSPY_ONE_STREAM_DEVICE_LOSS;
    }
    if (buffers->dma_backpressure_count != 0
        || queue->backpressure_count != 0) {
        status->flags |= AIRSPY_ONE_STREAM_CAPTURE_BACKPRESSURE;
    }
    status->captured_samples = buffers->next_sample_sequence;
    status->submitted_samples = queue->submitted_samples;
    status->retired_samples = queue->retired_samples;
    status->dropped_samples = buffers->dropped_samples;
    status->backpressure_events =
        buffers->dma_backpressure_count + queue->backpressure_count;
    status->queue_high_watermark = (uint16_t)queue->high_watermark;
    status->capture_buffer_count = (uint8_t)buffers->slot_count;
    status->descriptor_count = (uint8_t)queue->descriptor_count;
    status->completion_count = queue->completion_count;
    return 0;
}
