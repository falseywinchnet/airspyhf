#ifndef AIRSPY_STREAM_CONTRACT_H
#define AIRSPY_STREAM_CONTRACT_H

#include <stdint.h>

enum {
  AIRSPY_STREAM_CONTRACT_MAGIC = 0x53424f34u, /* "SBO4" */
  AIRSPY_STREAM_CONTRACT_VERSION = 10,
  AIRSPY_STREAM_BUFFER_COUNT = 10,
  AIRSPY_STREAM_RETIRE_QUEUE_COUNT = 16,
  AIRSPY_STREAM_GRANT_QUEUE_COUNT = 16,
  AIRSPY_STREAM_BUFFER_BYTES = 16 * 1024,
  AIRSPY_STREAM_MODE_LEGACY = 0,
  AIRSPY_STREAM_MODE_SYNTHETIC = 1,
  AIRSPY_STREAM_MODE_ADC_FOUR_BUFFER = 2,
  /* M4 halted capture; M0 is retiring transport ownership for a restart. */
  AIRSPY_STREAM_MODE_RECOVERING = 3,
  /* ADC phase is unknowable after FIFO loss; transport epoch is terminated. */
  AIRSPY_STREAM_MODE_POISONED = 4,
  AIRSPY_STREAM_BUFFER_FLAG_OVERWRITE_RISK = 1u << 0,
  AIRSPY_STREAM_BUFFER_FLAG_STEERING_DISCARD = 1u << 1,
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

  /*
   * M4 is the sole writer of granted_generation. M0 may attach a dTD only
   * after this field equals produced_generation. This removes the cross-core
   * race between M0 claiming a READY bank and M4 selecting it for DMA.
   */
  volatile uint32_t granted_generation;

  /* M0-owned transport and retirement fields. */
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
   * Single-producer/single-consumer retirement notification ring.
   * M0 writes entries and write_sequence after retiring a dTD. M4 consumes
   * entries and writes read_sequence at DMA boundaries.
   */
  volatile uint32_t retire_queue_write_sequence;
  volatile uint32_t retire_queue_read_sequence;
  volatile uint32_t retire_queue_overflows;
  volatile uint32_t retire_queue_entries[AIRSPY_STREAM_RETIRE_QUEUE_COUNT];

  /*
   * Mirrored single-producer/single-consumer grant notification ring.
   * M4 publishes granted_generation first, then the bank index and write
   * sequence. M0 submits exactly that bank rather than searching all records.
   */
  volatile uint32_t grant_queue_write_sequence;
  volatile uint32_t grant_queue_read_sequence;
  volatile uint32_t grant_queue_overflows;
  volatile uint32_t grant_queue_entries[AIRSPY_STREAM_GRANT_QUEUE_COUNT];

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
  volatile uint32_t stream_poisoned;
  volatile uint32_t stream_poison_count;
  volatile uint32_t poison_transport_terminated;
  volatile uint32_t adc_fifo_level_high_water;
  volatile uint32_t adc_fifo_full_observations;
  volatile uint32_t usb_system_error_count;

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

  /* Halt-free dynamic DMA steering telemetry. */
  volatile uint32_t steering_decisions;
  volatile uint32_t steering_overwrites;
  volatile uint32_t steering_overwrite_runs;
  volatile uint32_t steering_overwrite_run_current;
  volatile uint32_t steering_overwrite_run_maximum;
  volatile uint32_t steering_alternation_violations;
  volatile uint32_t steering_no_candidate_faults;
  volatile uint32_t steering_group_skips;
  volatile uint32_t steering_minimum_available;
  volatile uint32_t steering_minimum_groups;
  volatile uint32_t steering_current_available;
  volatile uint32_t steering_current_groups;
  volatile uint32_t steering_floor_boundaries;
  volatile uint32_t steering_floor_fast_path_boundaries;
  volatile uint32_t steering_full_scan_boundaries;
  volatile uint32_t steering_isr_cycles_maximum;
  volatile uint32_t maximum_capture_to_grant_age;
  volatile uint32_t stale_generation_completions;
  volatile uint32_t steering_available_histogram[
    AIRSPY_STREAM_BUFFER_COUNT + 1];

  airspy_stream_buffer_record_t buffers[AIRSPY_STREAM_BUFFER_COUNT];

  /* M0 refused to rewrite a dQH because quiescence was not proven. */
  volatile uint32_t usb_endpoint_configure_flush_failures;
} airspy_stream_contract_t;

static inline void airspy_stream_publish_barrier(void)
{
  __asm volatile ("dmb" ::: "memory");
}

#endif
