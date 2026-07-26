# Qualification of the ten-bank 12-bit packing proposal

Date: 2026-07-25

Status: **CPU-feasible, format-verified, transport-feasible on paper, not yet
qualified against the ADC FIFO.**

No production firmware was changed, built, or flashed during this analysis.
The current unpacked release remains the control.

## Executive result

The proposal is tangible.  It is not a fantasy algorithm and it is not likely
to lose on arithmetic alone:

- The existing firmware already contains a correct hand-written Cortex-M4
  packer with a 20-instruction inner loop per eight samples.
- A native golden test round-tripped one complete 16 KiB bank—8,192
  pseudorandom 12-bit samples—through the proposed in-place representation with
  no mismatch.
- With uncontended zero-wait SRAM, the existing loop should take roughly
  28–32 cycles per group, or 3.5–4.0 cycles per sample.  That is about
  28,700–32,800 cycles, 141–161 us, per bank at 204 MHz.
- The R2 supplies a bank every 409.6 us.  The estimated pack time consumes
  34–39% of that interval.  The proposed six-cycle-per-sample rejection line is
  49,152 cycles, 241 us, or 59% of the interval.
- Packing the R2 and Mini reduces their combined payload from an impossible
  64 MB/s to 48 MB/s.  That requires an average 11.71875 of the theoretical
  13 high-speed bulk transactions per microframe, leaving 1.28125 transactions
  per microframe, 5.248 MB/s, or 9.86% theoretical headroom.

The unresolved question is not compute throughput.  It is worst-case SRAM
arbitration latency at the ADCHS FIFO.  The packer performs 16 KiB of reads and
12 KiB of writes per bank: 70 MB/s of additional SRAM traffic while streaming.
The FIFO's approximately 0.8 us margin is sensitive to a short arbitration
delay, not merely average bandwidth.  An on-device mover test remains mandatory.

## Instruction-level result

Three code forms were examined with the current Cortex-M4 target:

| Kernel | Dynamic loop instructions per 8 samples | Observation |
|---|---:|---|
| Existing firmware hand assembly | 20 | Best current form |
| GCC 14.2.1 `-O2` C specimen | 22 | Compact, no unrolling |
| Apple Clang 16 `-O2` C specimen | 23 average | Four-way unrolled and much larger |

The existing hand assembly does exactly the required transform.  It loads four
words with `LDM`, constructs the three packed words with shifts, `UBFX`, `UXTH`,
and shifted `ORR` instructions, then stores three words with `STM`.

The 20 dynamic instructions undercount cycles because `LDM` and `STM` perform
seven SRAM word transfers and a taken loop branch refills the pipeline.  Using
28–32 cycles per group is a more defensible pre-measurement estimate than
equating instructions with cycles.

This corrects one optimistic statement in the proposal: at 20 MS/s raw input,
the expected M4 duty is approximately 35–40%, not 24%.  Packing may still be
well inside the deadline, but it is not thermally or electrically free.

## Memory-matrix result

The ten capture banks are distributed as follows:

| Slave group | Capture banks | Count |
|---|---|---:|
| S0, `0x10000000..0x1001ffff` | 1, 3, 5, 7 | 4 |
| S1, `0x10080000..0x10091fff` | 9, 2, 4, 6 | 4 |
| AHB bank at `0x20004000` | 0 | 1 |
| AHB bank at `0x20008000` | 8 | 1 |

The alternation rule keeps the bank being packed off the slave receiving the
current ADC writes.  That is necessary but not sufficient:

- M4 text and ordinary M4 state are in the S0 region.
- Whenever the ADC destination is also S0, packer instruction fetches can
  arbitrate with GPDMA even though the packer's data bank is elsewhere.
- M0, USB DMA, and a previously granted bank add a third concurrent access
  pattern that the two-bank “different slave” argument does not describe.

Therefore “different completed and active bank slaves” cannot by itself prove
FIFO neutrality.  The measured maximum arbitration delay is the load-bearing
result.

## USB and ownership work omitted by the proposal

The current ring path always schedules and validates a 16 KiB dTD:

```c
usb_transfer_schedule_tagged(
  &usb_endpoint_bulk_in,
  (void*)record->address,
  AIRSPY_STREAM_BUFFER_BYTES,
  adc_stream_retired,
  (void*)record,
  generation)
```

A packed ring bank is 12 KiB.  Enabling the packer without changing this would
send the stale 4 KiB tail, erase the bandwidth gain, and violate the host
format.  The packed ring needs a session-level 12 KiB payload contract used by
submission, retirement validation, diagnostics, and tests.

The current ISR also treats every produced, ungranted bank as eligible either
for USB grant or for oldest-bank overwrite.  A main-context packer needs an
explicit `PACKING` ownership state or mask.  That state must be excluded from:

- USB grant selection until packing is complete;
- DMA destination selection;
- the two-bank floor fast path;
- available-bank and available-slave floor accounting.

The safest sequencing preserves the rule that only the DMA ISR grants to M0:

1. ISR publishes a completed bank as raw-ready but not grantable.
2. M4 main context claims it with a tiny atomic state transition.
3. Main context packs it with interrupts enabled.
4. Main context publishes it as packed-ready.
5. The next DMA ISR grants packed-ready banks oldest-first.

This deliberately adds one boundary of grant latency without reopening the
main-loop/ISR stale-grant race fixed earlier.

A new packed-ring stream mode is clearer and safer than making
`use_packing` ambiguously mean either the legacy two-bank path or the ten-bank
path.

## Correct Phase 0 experiment

The mover must reproduce four loads and three stores per group.  A conventional
read-and-write-back mover performs 16 KiB of stores rather than the packer's
12 KiB and overstates write traffic by one third.

The experiment must also not write an active, USB-owned, or grantable bank.
There is no spare 16 KiB region in the production layout.  Controlled mover
configurations must reserve their source/destination test banks and run the
baseline with the same reduced effective ring depth.  For the deliberate
same-slave control, use a reserved bank on the same slave as the newly active
ADC destination; never write the active ADC bank itself.

For every configuration record:

- elapsed DWT cycles for a full 16-to-12 KiB mover pass;
- maximum pass cycles;
- ADC FIFO overflow delta;
- maximum DMA boundary interval and ISR execution cycles;
- active ADC slave, mover source slave, mover destination slave;
- captured, granted, overwritten, and minimum-reusable-bank counters.

Test matrix:

| Configuration | ADC vs mover data slave | Purpose |
|---|---|---|
| Reduced-ring baseline | no mover | Establish identical ownership/runway control |
| Packing-shaped in-place mover | different by alternation | Tests intended data placement |
| Packing-shaped cross-slave mover | all distinct where possible | Best-case matrix isolation |
| Packing-shaped same-slave mover | same, but never same bank | Required positive contention control |

Run first for seconds to catch catastrophic ownership or FIFO behavior, then
minutes, and only then the proposed ten-minute qualification.  Stop immediately
on a new FIFO overflow attributable to the mover.

## Decision

The experiment deserves an isolated diagnostic build.  Static qualification
says:

- **Format:** pass.
- **In-place overwrite geometry:** pass.
- **Core instruction budget:** likely pass with roughly 2.5–3x deadline margin.
- **Two-radio USB arithmetic:** pass on the theoretical scheduler, with only
  9.86% headroom.
- **Ring ownership integration:** feasible but requires explicit new state and
  12 KiB transport lengths.
- **FIFO safety:** unknown and decisive.

Do not expose packed-ring mode to the driver until the mover demonstrates zero
attributable FIFO overflows and the positive same-slave control proves that the
test is capable of detecting harmful contention.

