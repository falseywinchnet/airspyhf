#ifndef AIRSPY_STREAM_CONTRACT_H
#define AIRSPY_STREAM_CONTRACT_H

#include <stdint.h>

enum {
  AIRSPY_STREAM_CONTRACT_MAGIC = 0x53424f34u, /* "SBO4" */
  AIRSPY_STREAM_CONTRACT_VERSION = 5,
  AIRSPY_STREAM_BUFFER_COUNT = 10,
  AIRSPY_STREAM_BUFFER_BYTES = 16 * 1024,
  AIRSPY_STREAM_MODE_LEGACY = 0,
  AIRSPY_STREAM_MODE_SYNTHETIC = 1,
  AIRSPY_STREAM_MODE_ADC_FOUR_BUFFER = 2,
  /* M4 halted capture; M0 is retiring transport ownership for a restart. */
  AIRSPY_STREAM_MODE_RECOVERING = 3,
  AIRSPY_STREAM_BUFFER_FLAG_OVERWRITE_RISK = 1u << 0,
  AIRSPY_STREAM_GPDMA_IDLE = 0,
  AIRSPY_STREAM_GPDMA_PENDING = 1,
  AIRSPY_STREAM_GPDMA_PASSED = 2,
  AIRSPY_STREAM_GPDMA_ERROR = 3,
  AIRSPY_STREAM_GPDMA_TIMEOUT = 4
};

typedef struct {
  uint32_t address;

  /* M4-owned publication fields. produced_generation is written last. */
  volatile uint32_t produced_generation;
  volatile uint32_t dma_start_cycles;
  volatile uint32_t dma_complete_cycles;
  volatile uint32_t flags;

  /* M0-owned consumption fields. */
  volatile uint32_t submitted_generation;
  volatile uint32_t retired_generation;
  volatile uint32_t retired_bytes;
} airspy_stream_buffer_record_t;

typedef struct {
  /*
   * Preserve the legacy ABI at offsets 0, 4, and 8 while the new transport is
   * introduced behind an explicit mode.
   */
  volatile uint32_t legacy_usb_buffer_offset;
  volatile uint32_t legacy_usb_buffer_length;
  volatile uint32_t legacy_last_offset_m0;

  uint32_t magic;
  uint32_t version;
  volatile uint32_t mode;
  uint32_t buffer_count;
  uint32_t buffer_bytes;

  /* M4-owned capture telemetry. */
  volatile uint32_t capture_generation;
  volatile uint32_t capture_completed;
  volatile uint32_t capture_halted;
  volatile uint32_t overwrite_prevented;
  volatile uint32_t dma_error_count;
  volatile uint32_t ownership_overwrite_count;
  volatile uint32_t adc_fifo_overflow_count;
  volatile uint32_t adc_descriptor_error_count;
  volatile uint32_t adc_overrange_count;
  volatile uint32_t adc_underrange_count;
  volatile uint32_t minimum_completion_cycles;
  volatile uint32_t maximum_completion_cycles;

  /* M0-owned USB telemetry. */
  volatile uint32_t usb_submitted;
  volatile uint32_t usb_retired;
  volatile uint32_t usb_backpressure;
  volatile uint32_t usb_partial;
  volatile uint32_t usb_errors;
  volatile uint32_t usb_cancelled;
  volatile uint32_t usb_suspended;
  volatile uint32_t usb_suspend_count;
  volatile uint32_t usb_resume_count;
  volatile uint32_t usb_suspend_discontinuities;
  volatile uint32_t usb_queue_recovery_count;
  volatile uint32_t usb_partial_discontinuity_count;
  volatile uint32_t usb_controller_error_irq_count;
  volatile uint32_t usb_bus_reset_count;
  volatile uint32_t usb_port_change_count;

  /*
   * Recoverable capture restart handshake.
   * M4 writes request_generation while DMA is halted, then publishes
   * AIRSPY_STREAM_MODE_RECOVERING last.
   * M0 flushes dTD ownership, resumes the endpoint, and publishes ack.
   * M4 rebuilds ADC/DMA and publishes completed_generation.
   */
  volatile uint32_t recovery_request_generation;
  volatile uint32_t recovery_acknowledged_generation;
  volatile uint32_t recovery_completed_generation;
  volatile uint32_t dma_recovery_count;
  volatile uint32_t dma_recovery_failure_count;
  volatile uint32_t dma_recovery_dropped_buffer_estimate;
  volatile uint32_t last_dma_error_status;
  volatile uint32_t backpressure_discontinuity_count;

  /*
   * M0 writes command_generation and destination_index, then signals M4.
   * M4 writes the remaining result fields and publishes completed_generation
   * last.
   */
  volatile uint32_t gpdma_command_generation;
  volatile uint32_t gpdma_completed_generation;
  volatile uint32_t gpdma_destination_index;
  volatile uint32_t gpdma_status;
  volatile uint32_t gpdma_bytes;
  volatile uint32_t gpdma_cycles;
  volatile uint32_t gpdma_error_status;
  volatile uint32_t gpdma_expected_checksum;
  volatile uint32_t gpdma_actual_checksum;

  /* Fixed-size clock and CPU-work telemetry. */
  volatile uint32_t clock_stream_pll1_ctrl;
  volatile uint32_t clock_idle_pll1_ctrl;
  volatile uint32_t clock_base_m4;
  volatile uint32_t clock_base_periph;
  volatile uint32_t clock_base_apb1;
  volatile uint32_t clock_base_apb3;
  volatile uint32_t clock_high_transitions;
  volatile uint32_t clock_low_transitions;
  volatile uint32_t m4_dma_isr_cycles_total;
  volatile uint32_t m4_dma_isr_cycles_maximum;
  volatile uint32_t m4_dma_isr_count;
  volatile uint32_t m4_resume_cycles_total;
  volatile uint32_t m4_resume_cycles_maximum;
  volatile uint32_t m4_resume_count;
  volatile uint32_t m0_submit_cycles_total;
  volatile uint32_t m0_submit_cycles_maximum;
  volatile uint32_t m0_submit_count;
  volatile uint32_t m0_retire_cycles_total;
  volatile uint32_t m0_retire_cycles_maximum;
  volatile uint32_t m0_retire_count;

  airspy_stream_buffer_record_t buffers[AIRSPY_STREAM_BUFFER_COUNT];
} airspy_stream_contract_t;

static inline void airspy_stream_publish_barrier(void)
{
  __asm volatile ("dmb" ::: "memory");
}

#endif
