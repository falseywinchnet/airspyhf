# Airspy R2/Mini firmware longevity audit

Date: 2026-07-23

Scope: static source review only. No device was accessed, flashed, restarted, or
exercised. The source hardening described below remains unqualified until it is
built and deliberately tested on hardware.

Hardening status: two corrective source passes were applied after this audit.
See `2026-07-23-hardening-pass-1.md` and
`2026-07-23-hardening-pass-2-recoverable-streaming.md`. Findings below describe
the audit baseline. GPDMA now has a drop-and-restart handshake; EP0,
command-publication, stale-retirement, suspend observability, USB hardware wait
deadlines, and cross-core acknowledgement deadlines are corrected in source but
remain hardware-unqualified.

## Executive result

The new ten-bank ADC-to-USB ring has a sound ownership model for ordinary
continuous operation and backpressure. Its generation counter and cycle-counter
wraps are not latent long-run failures. The current source is nevertheless not
yet suitable for a claim of unattended, indefinite recovery. One definite
production-build interrupt-storm defect remains, and several hardware waits
have no timeout or recovery policy.

## Definite high-severity defect

### GPDMA errors are cleared only in a debug build

`airspy_m4/airspy_m4.c:dma_isr()` reads and clears `INTERRSTAT` inside
`#ifdef DMA_ISR_DEBUG`. In a normal build, the handler processes terminal-count
interrupts but never acknowledges a DMA error interrupt.

Consequence: one rare channel error can leave the DMA interrupt asserted,
causing immediate repeated entry into `dma_isr()`. The M4 can be consumed by the
interrupt indefinitely while capture and M0 command service cease making useful
progress.

Required correction: production code must always read and clear channel 0's
error status. It should record the fault, stop or reset channel 0, invalidate the
current capture epoch, and make a deliberate choice between a bounded restart
and returning the receiver to OFF. Merely clearing the bit and continuing the
ring would conceal potentially corrupt or discontinuous sample data.

## High-severity recovery gaps

### USB hardware waits are unbounded

Endpoint priming, endpoint flushing, controller reset, and initial scheduling
contain loops with no deadline. A controller or PHY state that fails to make the
expected transition can therefore hold the M0 forever, commonly from within the
USB interrupt handler.

Affected paths include:

- waiting for `ENDPTPRIME` to clear;
- waiting for `ENDPTFLUSH` to clear;
- waiting for `ENDPTSTAT` to become idle before priming;
- waiting for the controller reset bit to clear;
- waiting for the ATDTW tripwire to latch.

Recovery cannot be added independently to each loop. These paths need one
controller-level policy: bounded wait, capture producer stopped first, all
software ownership retired as cancelled, controller reset/reinitialization, and
either re-enumeration or a clean OFF state.

### Cross-core commands have no deadline

The M0 waits forever for M4 acknowledgements of prepare/start/stop, sample-rate,
and packing commands. A fault, lost event, clock transition failure, or wedged
peripheral on the M4 side therefore also wedges USB control handling on M0.

The shared command words need a transaction epoch plus a bounded deadline.
Timeout recovery must not reuse a buffer until both cores agree that DMA has
stopped.

### Control-transfer allocation can spin inside the USB ISR

`usb_transfer_schedule_block()` retries forever when a control endpoint's dTD
pool is exhausted. Its callers execute during USB request processing. If
completion processing required to return a dTD cannot run until that same ISR
finishes, the retry is a self-deadlock.

New SETUP reception should explicitly abort/retire the prior EP0 transaction,
and EP0 scheduling should fail into a controlled stall/reset rather than spin.

## Medium-severity gaps

### USB suspend is ignored

Suspend is acknowledged at the controller status level but has no application
policy. Capture may fill the ring, halt safely at backpressure, and later resume,
but the ADC/tuner remain powered and a long suspension can generate FIFO
overflow telemetry and an unavoidable discontinuity.

Decide and document whether suspend means:

1. preserve the stream briefly and tolerate a discontinuity;
2. stop capture and require a host restart; or
3. enter a bounded grace period followed by OFF.

### Peripheral setup and flash operations contain infinite waits

PLL lock, ADCHS reset/status/FIFO empty, I2C transactions, and SPI-flash busy
polling include unbounded waits. Most occur at startup or reconfiguration rather
than in the continuous hot path, but they prevent autonomous recovery after a
peripheral fault.

Each needs a device-specific deadline and a failure result propagated to the
control request. Adding a generic watchdog first would turn these into opaque
reboot loops rather than diagnosed failures.

### Fault handlers deliberately stop forever

HardFault, MemManage, BusFault, and UsageFault handlers retain debugging state
and spin. This is useful during development but provides no deployed recovery.
A later production policy should persist a compact reset reason and restart the
whole device. A watchdog is appropriate only after normal long-running paths
have explicit progress markers, so it cannot reset healthy backpressure or an
intentionally idle receiver.

## Current-source hardening already present

The following source changes close real ownership and state-machine holes, but
have not been hardware-qualified as part of this audit:

- repeated RX and ARMED requests are idempotent;
- all unsafe active-state transitions pass through OFF;
- the ADC producer is stopped before USB dTD ownership is flushed;
- USB bus reset notifies the application and stops capture before endpoint
  queues are reset;
- packing and sample-rate changes stop and reconstruct the complete
  producer/transport epoch;
- M4 publishes legacy mode after stopping ADC/DMA, preventing further ring
  submission into a disabled endpoint;
- a non-control dTD error flushes later descriptors that the halted endpoint
  would otherwise strand.

These changes improve restart and error-boundary behavior. They do not solve the
unbounded hardware waits or the production DMA-error interrupt problem above.

## Ring properties that are safe by inspection

### Backpressure and overwrite prevention

The M4 checks the bank two positions ahead before DMA can enter it. If USB still
owns that bank, channel 0 is halted after the already in-flight bank. M4 resumes
only after `produced_generation`, `submitted_generation`, and
`retired_generation` agree. This chooses a visible capture discontinuity over
silent memory overwrite.

### Generation wrap

Generation zero is reserved and skipped. At 10 MS/s with 16 KiB banks, a
32-bit generation wraps on the order of forty days. Equality remains safe:
there are only ten banks and capture halts rather than allowing a record to
remain outstanding across an entire 32-bit epoch. Telemetry counters also wrap,
but they are not used as ownership control state.

### Cycle-counter wrap

The 32-bit DWT cycle counter wraps roughly every 30.7 seconds at 140 MHz.
Elapsed time is computed by unsigned subtraction between adjacent bank
completions, whose interval is far below one wrap, so the timing calculation is
valid across wrap.

### Cross-core publication ordering

The ownership records publish their generation after their associated fields
with a data-memory barrier. Event signalling uses a data-synchronization barrier
before `SEV`. The consumer reads the generation first and observes the published
record under the intended single-producer/single-consumer ownership scheme.

## Recommended hardening order

1. Implement production GPDMA-error acknowledgement and a clean capture-fault
   state.
2. Replace EP0 blocking allocation with explicit transaction abort/failure.
3. Introduce bounded USB-controller waits with one coherent reset path.
4. Add cross-core command epochs and deadlines.
5. Define suspend/disconnect behavior.
6. Add bounded PLL, ADC, I2C, and flash waits with propagated error results.
7. Only then add a windowed watchdog driven by independent M0 and M4 progress
   evidence.

The first item is a local correctness repair. Items 2 through 7 change recovery
semantics and should be implemented in small, separately reviewable steps before
any long-duration qualification run.
