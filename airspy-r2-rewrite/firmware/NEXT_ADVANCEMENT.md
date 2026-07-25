# Firmware next advancement

Status: current engineering handoff  
Baseline: V5c universal R2/Mini, alternating-SRAM ten-bank ring  
Wire compatibility: unchanged legacy Airspy One API and headerless sample stream

## Why this is next

V5c makes buffer and dTD ownership explicit and gives short USB congestion
several milliseconds of runway. Its remaining congestion policy is wrong for a
continuous receiver: when USB owns the bank that DMA would reuse, firmware
halts capture. That converts a host transport delay into ADC/FIFO disruption
and can require GPDMA recovery. It also retains old samples for the duration of
an arbitrarily long host stall.

Vanilla avoids a deep stale queue by continuously rotating its two capture
banks, but it does so without explicit USB ownership and can silently skip or
tear history. The next design keeps V5c's ownership rigor while recovering the
useful vanilla property: severe congestion loses continuity but remains close
to current RF time.

The policy is:

> Absorb short stalls losslessly. Once the latency budget is exhausted, never
> halt ADC because USB is late; overwrite private capture history and resume
> USB from recent samples.

## Fixed timing facts

At nominal 10 MS/s complex output, the ADC produces 20 million real samples/s
in 16-bit containers:

```text
40 MB/s unpacked
16 KiB bank = 409.6 microseconds
eight transport banks = 3.2768 milliseconds nominal
two private banks = 819.2 microseconds of rolling recency
```

This device-side latency is small relative to the established host transfer and
application queues. It is useful USB-scheduling runway, not permission for
unbounded stale data.

## Chosen architecture

Partition the qualified ten banks without adding unproven memory:

```text
normal transport ring:
    8 banks = 4 local-SRAM1 + 4 other-slave banks

private rolling ring:
    2 banks = 1 local-SRAM1 + 1 other-slave bank
```

The private pair is never referenced by a USB dTD.

Both rings retain the alternating-slave rule. V5/V5b demonstrated one
deterministic ADC FIFO event per ten-bank revolution when two consecutive DMA
destinations used local SRAM1. V5c removed it by alternating five local-SRAM1
banks with five destinations on other SRAM slaves. The required property is
alternation, not the number five.

## Normal operation

1. GPDMA captures through the eight-bank transport ring.
2. M0 submits completed banks through the linked bulk-IN dTD queue.
3. Short host congestion accumulates valid, ordered banks.
4. USB retirement releases the exact generation of each bank.
5. The private pair remains free of USB ownership at all times.

Normal operation must not add packing, copying, framing, or a new public
request. The raw byte stream remains compatible with existing libairspy.

## Linchpin transition

The transition is requested before DMA would encounter a transport bank still
owned by USB. It must be based on prospective ownership/age, not an ADC FIFO
overflow after the fact.

At a completed bank boundary:

1. stop publishing new transport banks to USB;
2. change a sufficiently future GPDMA LLI link, never the active LLI;
3. let GPDMA continue directly into the private two-bank closed chain;
4. start a new transport/ownership epoch;
5. retire, cancel, or flush stale USB work independently.

The ADC channel is not disabled, paused, or reconstructed for ordinary USB
backpressure. The word “halted” must disappear from the host-congestion path.

The exact trigger occupancy—probably six or seven of eight transport
banks—is a measured parameter, not yet frozen. It must leave enough boundary
margin for deterministic chain switching while keeping the total firmware
latency below the selected budget.

## Rolling mode

While USB remains unavailable:

- GPDMA continuously alternates across the private pair;
- completing a new bank intentionally overwrites older private history;
- capture-generation and discarded-bank counters continue advancing;
- tuner, clocks, ADC configuration, and requested sample rate remain unchanged;
- USB recovery may fail, retry, or escalate without stopping capture.

This is controlled loss of history, not USB-owned memory corruption. No private
bank may acquire a dTD while it remains part of the rolling chain.

## USB controller policy

Minimize controller states:

```text
normal:
    linked ACTIVE bulk-IN queue

severe congestion:
    stop append -> one bounded detach/flush attempt -> endpoint idle

return:
    one new ownership epoch -> one prime -> linked ACTIVE queue
```

Do not repeatedly reset, flush, prime, clear halt, or reset DATA0/DATA1 while
trying to recover ordinary congestion. Application pause/resume preserves the
shared data toggle. USB bus reset and endpoint-halt recovery remain distinct
USB protocol boundaries.

Failure to flush USB must pin the affected transport banks, but it must not
affect the private capture pair or stop ADC.

## Detecting host return

Preferred experiment: validate the ChipIdea endpoint-NAK status/interrupt as
evidence that the host has resumed issuing bulk-IN tokens while the endpoint is
unprimed.

If reliable:

1. observe renewed IN polling;
2. at a private-bank completion boundary, redirect GPDMA to the eight-bank
   transport chain;
3. intentionally discard the private rolling history;
4. publish the first newly completed transport bank in the new epoch;
5. prime USB and continue.

This resumes no more than approximately one transport-bank interval behind
live capture without ever giving USB ownership of a private bank.

The fallback is a carefully defined probe transfer, but it is less desirable
because a queued probe pins memory and may expose a stale block. No ZLP, marker,
header, or new stream framing is introduced merely to detect liveness.

## Ownership and ordering invariants

- USB never references the private rolling pair.
- DMA never enters a transport bank owned by a live or stale dTD.
- A stale completion may release only its recorded old generation.
- Epoch change prevents old completions from submitting or retiring new data.
- Chain switching occurs only at a proven DMA boundary.
- Ambiguous ownership remains a safe-stop/reset class fault; ordinary host
  congestion does not.
- A discontinuity is preferable to torn, reordered, or silently aliased data.

## Telemetry additions

Extend the existing private request `0x87`, versioning the structure if its
layout changes. Add:

- rolling-mode entries;
- normal-to-private and private-to-normal transition counts;
- trigger occupancy and maximum observed occupancy;
- banks deliberately discarded during rolling mode;
- time/banks spent in rolling mode;
- USB detach/flush attempts, successes, failures, and timeouts;
- host-return/NAK detections;
- reentry successes and failures;
- stale-epoch completions;
- maximum capture-to-submit age.

Counters remain observational. They must not add blocking work to the capture
path and must use unsigned wrap-safe deltas.

## What this does not promise

“Never halt for USB congestion” does not mean the ADC can continue through a
real GPDMA channel fault, corrupted LLI, lost clock, impossible ownership
state, explicit receiver stop, or MCU reset. Those faults may require bounded
reconstruction or safe failure.

The promise is narrower and testable:

> A late or unavailable host, dTD exhaustion, and ordinary bulk-IN
> backpressure do not halt ADC/GPDMA capture.

## Deferred work

- Lossless packing remains disabled by default until full-bank worst-case cycle
  and SRAM-arbitration measurements pass.
- Clang/`-Oz`, code relocation, and possible twelve-bank memory reclamation are
  separate experiments. They are not required for this design.
- New framing, sequence headers, compression, sample rates, and clock changes
  are outside this advancement.
- Additional SRAM at `0x18000000` remains unqualified.

## Implementation order

1. Express the eight-plus-two rings and allowed ownership states in the host
   model.
2. Add transition and stale-epoch tests, including every bank-boundary phase.
3. Generate two closed GPDMA chains with linker/address assertions.
4. Implement future-LLI switching without channel disable.
5. Redirect existing backpressure handling to rolling mode.
6. Separate USB detach/recovery completely from ADC recovery.
7. Validate endpoint-NAK host-return detection on R2 and Mini.
8. Implement reentry and telemetry.
9. Inspect generated assembly and worst-case transition cycles.
10. Build an experimental image only after the model and static contracts pass.

## Exit criteria

- No ADC channel halt caused by USB congestion in source or generated traces.
- No USB dTD ever references a private rolling bank.
- No transport bank is overwritten while USB-owned.
- Short stalls below the trigger remain lossless and ordered.
- Long stalls produce counted discontinuities while capture generations
  continue monotonically.
- Reentry begins with recent data and bounded firmware age.
- USB detach failure leaves capture running in the private pair.
- R2 and Mini pass repeated congestion/recovery and start/stop cycles.
- Stock libairspy continues receiving the unchanged raw stream.
