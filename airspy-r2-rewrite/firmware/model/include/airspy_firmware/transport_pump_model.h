#ifndef AIRSPY_FIRMWARE_TRANSPORT_PUMP_MODEL_H
#define AIRSPY_FIRMWARE_TRANSPORT_PUMP_MODEL_H

#include "airspy_firmware/buffer_model.h"
#include "airspy_firmware/usb_queue_model.h"

#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct airspy_transport_pump_report {
    size_t submitted_buffers;
    int endpoint_primed;
    int waiting_for_descriptor;
} airspy_transport_pump_report;

airspy_usb_queue_result airspy_transport_pump(
    airspy_buffer_ring* buffers,
    airspy_usb_queue* queue,
    airspy_transport_pump_report* report);

#ifdef __cplusplus
}
#endif

#endif
