# Airspy R2/Mini firmware hardening, pass 2: burp and continue

Date: 2026-07-23

Status: static implementation and compile checks only. No radio was accessed,
flashed, restarted, or run. This source is newer than, and is not yet a
replacement for, the live-qualified V4 image.

## Intended behavior

A recoverable stream fault should cost a short discontinuity, not the receiver
session. The firmware now distinguishes four cases:

| Condition | Action |
| --- | --- |
| USB queue pressure that fits in the ten-bank ring | Absorb it; no sample-loss claim |
| USB dTD error or short transfer | Retire/cancel affected owners, re-prime, continue |
| GPDMA channel-0 error | Halt capture, drop the uncertain epoch, rebuild ADC/DMA, continue |
| Broken ownership invariant or stuck hardware handshake | Remain halted safely or reset the whole MCU |

The last row is deliberately narrow. Continuing after the firmware can no
longer prove who owns a memory bank would risk silent overwrite and corrupted,
out-of-order samples.

## GPDMA recovery handshake

The M4 interrupt handler always acknowledges GPDMA error status in production.
For a channel-0 error it halts the channel, records the raw status, allocates a
new recovery generation, publishes `RECOVERING`, and wakes M0. A coincident
terminal-count interrupt is acknowledged but its bank is not advertised as a
valid completion.

M0 then:

1. disables and flushes only bulk IN, leaving tuner and receiver state alone;
2. receives cancellation callbacks for every submitted dTD;
3. explicitly drops a completed bank that had not yet acquired a dTD;
4. verifies that every bank is free before acknowledging the recovery;
5. resumes bulk IN without resetting DATA0/DATA1.

Only after that acknowledgement does M4 reset and reconstruct ADC and channel
0, preserving the requested sample rate, gains, frequency, tuner state, and
accumulated diagnostics. The stream resumes in a new capture generation. The
host sees a gap of an estimated one or more 16 KiB banks rather than a receiver
shutdown.

If a submitted bank is still owned after the hardware flush, M0 refuses to
acknowledge. That failure is recorded once for the recovery generation and the
producer remains halted. This is the safe-state boundary.

## USB queue recovery

A completed bulk dTD carrying HALTED, BUFFER_ERROR, or TRANSACTION_ERROR is
retired with its actual status. Later active dTDs on the halted hardware queue
are cancelled and released; the next producer submission primes the now-empty
endpoint and clears the queue-head halt state.

A successful short transfer is also recorded as a discontinuity. Ordinary dTD
pool exhaustion remains nonblocking on the ADC ring: M0 records backpressure
and returns to its event loop. M4 halts before overwriting an owned bank and
resumes when ownership clears.

The compatibility scheduling helper no longer spins on pool exhaustion either:
control transfers stall until the next SETUP boundary, and non-control callers
drop that work item. Impossible queue-registration invariants now take the
last-resort reset path instead of hanging M0 forever.

Backpressure alone is not counted as lost RF time. The separate
`backpressure_discontinuity_count` advances only when the ADC FIFO overflow bit
shows that the pause actually lost samples.

## Cross-core and controller deadlines

M0 publishes each M4 command and its argument in one aligned 32-bit store.
Prepare/start/stop, sample-rate, and packing acknowledgements now have a
SysTick deadline of eight million AHB cycles: about 39 ms at 204 MHz or 67 ms
at 120 MHz. A missed deadline means M0 cannot safely revoke unknown M4/DMA
state, so it performs a whole-MCU core reset.

The same bounded deadline protects USB peripheral reset, controller reset,
endpoint prime/flush waits, endpoint-idle scheduling, and the ATDTW tripwire
latch. These waits represent local hardware state transitions; they are not
waiting for the host to accept bulk packets. Normal USB contention therefore
does not consume this deadline or cause a reset.

## Finding the source of an audible catch

The stream telemetry contract is now version 5. It independently counts:

- ADC FIFO overflow, descriptor error, and over/under-range flags;
- GPDMA errors, completed recoveries, failed recoveries, and estimated dropped
  banks;
- USB dTD errors, queue recoveries, cancellations, short transfers, and queue
  backpressure;
- raw USB controller-error interrupts, bus resets, and port changes;
- discontinuities proven to result from backpressure or suspend;
- request, acknowledgement, and completion generations for recovery.

The host reader supports a low-rate sidecar mode:

```text
hardware/tools/adc_ring_telemetry --watch
hardware/tools/adc_ring_telemetry --watch --interval-ms 250
```

These event counters live from firmware boot to firmware reset; starting a new
capture no longer lets M4 erase counters owned by M0. That both removes a
cross-core write race and preserves evidence when an application restarts after
a USB event.

The reader prints deltas while an SDR application is streaming. A catch coincident with
an ADC FIFO counter is capture-side loss; a DMA recovery identifies GPDMA; a
dTD/partial counter identifies USB transport; backpressure without the
discontinuity counter was absorbed by the ring. If all firmware counters remain
clean, the remaining likely domains are the host driver, DSP/audio pipeline, or
the received signal itself.

The control read is 608 bytes per interval. That diagnostic traffic is tiny
relative to the stream but is not literally zero perturbation.

## Compile result

A clean universal build completed with Arm GNU Toolchain 14.2.1:

```text
M0: text 11588, data 712, bss 3712
M0s: text 424, data 0, bss 0
M4 image: text 20336, data 860, bss 400
M0 binary SHA-256:
82ff92afde1b5ee60fa6a40c2f13d8d6ea2723c471bc85a461356ab1b320821e
Current universal binary SHA-256:
138d64f1a3dd4007f813264f721435729b6180cd5ff0851baa36939586b888d5
```

The only linker warning is the inherited M0 load segment with read, write, and
execute permissions. The host telemetry reader also compiles cleanly with the
system C compiler and libusb. To avoid a 2 KiB alignment hole in M0's nearly
full 16 KiB execution SRAM, the 2 KiB-aligned endpoint queue-head array occupies
its own 2 KiB AHB1 window at `0x20003800`. The dTD pool remains in the
USB-DMA-visible metadata bank at `0x1001c000`.

## R2 live SRAM-order correction

The first live V5 run produced 12,872 ADC FIFO flags in 128,727 banks: almost
exactly one per ten-bank revolution, while USB queue backpressure, dTD errors,
host drops, partial transfers, and ownership halts remained zero. Moving the
queue heads out of local SRAM did not change that ratio.

The ring contained five destinations in local SRAM1 and five on other slaves,
but its final two destinations were both local SRAM1. Reordering the same ten
addresses into strict local1/other alternation removed the deterministic event.
The corrected live run advanced through thousands of banks with zero FIFO,
USB, ownership, DMA, recovery, or host-drop counters.

The backpressure-discontinuity counter was also corrected: a generic ADC FIFO
flag no longer increments it. It now advances only when FIFO overflow is
observed while resuming from an actual ownership/backpressure halt.

## Deliberately unfinished

- ADC reset, FIFO-empty, PLL, I2C, and flash waits still need device-specific
  deadlines and propagated failures.
- CPU exception handlers still retain state and stop for debugging rather than
  persisting a reset reason.
- Last-resort resets do not yet preserve their cause across reset.
- The new recovery paths have not been fault-injected or qualified on hardware.

Those limits matter for a future “months unattended” claim. This pass narrows
ordinary streaming failures to drop-and-continue behavior without pretending
that an unprovable ownership state is recoverable.
