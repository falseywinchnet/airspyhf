/*
 * Airspy One Wine bridge protocol.
 *
 * A Windows x86_64 airspy.dll shim talks to a native macOS helper over
 * loopback.  Control calls are synchronous.  Sample frames use a second
 * connection so a blocked callback cannot corrupt the control stream.
 */
#ifndef AIRSPY_ONE_BRIDGE_PROTO_H
#define AIRSPY_ONE_BRIDGE_PROTO_H

#include <stdint.h>

#define AOB_DEFAULT_PORT      53978
#define AOB_PORT_ENV          "AIRSPY_BRIDGE_PORT"
#define AOB_CHANNEL_CONTROL   0x4354524cu /* CTRL */
#define AOB_CHANNEL_DATA      0x44415441u /* DATA */
#define AOB_DATA_FRAME_MAGIC  0x41535031u /* ASP1 */

enum aob_op {
    AOB_OP_LIB_VERSION = 1,
    AOB_OP_INIT,
    AOB_OP_EXIT,
    AOB_OP_LIST_DEVICES,
    AOB_OP_OPEN,
    AOB_OP_OPEN_SN,
    AOB_OP_CLOSE,
    AOB_OP_GET_SAMPLERATES,
    AOB_OP_SET_SAMPLERATE,
    AOB_OP_START_RX,
    AOB_OP_STOP_RX,
    AOB_OP_IS_STREAMING,
    AOB_OP_SET_SAMPLE_TYPE,
    AOB_OP_SET_FREQ,
    AOB_OP_SET_LNA_GAIN,
    AOB_OP_SET_MIXER_GAIN,
    AOB_OP_SET_VGA_GAIN,
    AOB_OP_SET_LNA_AGC,
    AOB_OP_SET_MIXER_AGC,
    AOB_OP_SET_LINEARITY_GAIN,
    AOB_OP_SET_SENSITIVITY_GAIN,
    AOB_OP_SET_RF_BIAS,
    AOB_OP_SET_PACKING,
    AOB_OP_BOARD_ID_READ,
    AOB_OP_VERSION_STRING_READ,
    AOB_OP_BOARD_PARTID_SERIALNO_READ,
    AOB_OP_SI5351C_WRITE,
    AOB_OP_SI5351C_READ,
    AOB_OP_R820T_WRITE,
    AOB_OP_R820T_READ,
    AOB_OP_GPIO_WRITE,
    AOB_OP_GPIO_READ,
    AOB_OP_GPIODIR_WRITE,
    AOB_OP_GPIODIR_READ,
    AOB_OP_SPIFLASH_ERASE,
    AOB_OP_SPIFLASH_ERASE_SECTOR,
    AOB_OP_SPIFLASH_WRITE,
    AOB_OP_SPIFLASH_READ,
    AOB_OP_GET_MONITOR_SNAPSHOT
};

enum aob_monitor_event {
    AOB_MONITOR_HELPER_STARTED = 1,
    AOB_MONITOR_DEVICE_OPENED,
    AOB_MONITOR_STREAM_STARTED,
    AOB_MONITOR_STREAM_STOPPED,
    AOB_MONITOR_SAMPLERATE_CHANGED,
    AOB_MONITOR_DEVICE_CLOSED
};

#pragma pack(push, 1)
typedef struct {
    uint32_t op;
    uint64_t dev;
    uint32_t in_len;
} aob_req_hdr;

typedef struct {
    int32_t ret;
    uint32_t out_len;
} aob_resp_hdr;

typedef struct {
    uint64_t dev;
} aob_data_hello;

typedef struct {
    uint32_t magic;
    uint32_t sample_count;
    uint64_t dropped_samples;
    uint32_t sample_type;
    uint32_t payload_bytes;
} aob_data_hdr;

#define AOB_MONITOR_MAGIC 0x414f4235u /* AOB5 */
#define AOB_MONITOR_VERSION 1u
#define AOB_STREAM_CONTRACT_VERSION 6u
#define AOB_STREAM_BUFFER_COUNT 10u

typedef struct {
    uint32_t address;
    uint32_t produced_generation;
    uint32_t dma_start_cycles;
    uint32_t dma_complete_cycles;
    uint32_t flags;
    uint32_t granted_generation;
    uint32_t submitted_generation;
    uint32_t retired_generation;
    uint32_t retired_bytes;
} aob_stream_buffer_record;

typedef struct {
    uint32_t legacy[3];
    uint32_t magic;
    uint32_t version;
    uint32_t mode;
    uint32_t buffer_count;
    uint32_t buffer_bytes;
    uint32_t capture_generation;
    uint32_t capture_completed;
    uint32_t capture_halted;
    uint32_t overwrite_prevented;
    uint32_t dma_error_count;
    uint32_t ownership_overwrite_count;
    uint32_t adc_fifo_overflow_count;
    uint32_t adc_descriptor_error_count;
    uint32_t adc_overrange_count;
    uint32_t adc_underrange_count;
    uint32_t minimum_completion_cycles;
    uint32_t maximum_completion_cycles;
    uint32_t usb_submitted;
    uint32_t usb_retired;
    uint32_t usb_backpressure;
    uint32_t usb_partial;
    uint32_t usb_errors;
    uint32_t usb_cancelled;
    uint32_t usb_suspended;
    uint32_t usb_suspend_count;
    uint32_t usb_resume_count;
    uint32_t usb_suspend_discontinuities;
    uint32_t usb_queue_recovery_count;
    uint32_t usb_partial_discontinuity_count;
    uint32_t usb_controller_error_irq_count;
    uint32_t usb_bus_reset_count;
    uint32_t usb_port_change_count;
    uint32_t recovery_request_generation;
    uint32_t recovery_acknowledged_generation;
    uint32_t recovery_completed_generation;
    uint32_t dma_recovery_count;
    uint32_t dma_recovery_failure_count;
    uint32_t dma_recovery_dropped_buffer_estimate;
    uint32_t last_dma_error_status;
    uint32_t backpressure_discontinuity_count;
    uint32_t gpdma[9];
    uint32_t clock_stream_pll1_ctrl;
    uint32_t clock_idle_pll1_ctrl;
    uint32_t clock_base_m4;
    uint32_t clock_base_periph;
    uint32_t clock_base_apb1;
    uint32_t clock_base_apb3;
    uint32_t clock_high_transitions;
    uint32_t clock_low_transitions;
    uint32_t m4_dma_isr_cycles_total;
    uint32_t m4_dma_isr_cycles_maximum;
    uint32_t m4_dma_isr_count;
    uint32_t m4_resume_cycles_total;
    uint32_t m4_resume_cycles_maximum;
    uint32_t m4_resume_count;
    uint32_t m0_submit_cycles_total;
    uint32_t m0_submit_cycles_maximum;
    uint32_t m0_submit_count;
    uint32_t m0_retire_cycles_total;
    uint32_t m0_retire_cycles_maximum;
    uint32_t m0_retire_count;
    uint32_t steering_decisions;
    uint32_t steering_overwrites;
    uint32_t steering_overwrite_runs;
    uint32_t steering_overwrite_run_current;
    uint32_t steering_overwrite_run_maximum;
    uint32_t steering_alternation_violations;
    uint32_t steering_no_candidate_faults;
    uint32_t steering_group_skips;
    uint32_t steering_minimum_available;
    uint32_t steering_minimum_groups;
    uint32_t steering_floor_boundaries;
    uint32_t steering_isr_cycles_maximum;
    uint32_t maximum_capture_to_grant_age;
    uint32_t stale_generation_completions;
    uint32_t steering_available_histogram[AOB_STREAM_BUFFER_COUNT + 1];
    aob_stream_buffer_record buffers[AOB_STREAM_BUFFER_COUNT];
} aob_stream_telemetry;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t telemetry_valid;
    int32_t telemetry_result;
    uint64_t session_epoch;
    uint32_t event;
    uint32_t streaming;
    uint32_t sample_rate;
    uint64_t host_dropped_samples;
    aob_stream_telemetry telemetry;
} aob_monitor_snapshot;
#pragma pack(pop)

#if !defined(_MSC_VER)
_Static_assert(sizeof(aob_stream_telemetry) == 748,
               "bridge and firmware stream contracts must agree");
_Static_assert(sizeof(aob_monitor_snapshot) == 792,
               "monitor wire layout changed unexpectedly");
#endif

#endif
