#ifndef AIRSPY_FIRMWARE_STREAM_STATUS_MODEL_H
#define AIRSPY_FIRMWARE_STREAM_STATUS_MODEL_H

#include "airspy_firmware/buffer_model.h"
#include "airspy_firmware/usb_queue_model.h"
#include "airspy_one/protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

int airspy_stream_status_snapshot(
    const airspy_buffer_ring* buffers,
    const airspy_usb_queue* queue,
    int stream_running,
    airspy_one_stream_status* status);

#ifdef __cplusplus
}
#endif

#endif
