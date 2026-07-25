# Firmware next advancement

Status: V7 release build flashed and byte-verified on R2 and Mini
Baseline: ten-bank steered ring, M4-only grants, M0 retirement ring
Wire compatibility: unchanged legacy Airspy One API and headerless sample stream

## Implementation result

All approved items landed in V7:

- bank placement is 4/4/1/1 across S1/S2/S3/S4 with circular slave
  alternation; M4 data and both shared mailboxes occupy the vacated S1 window,
  while the 1.25 KiB dTD pool occupies the qualified 8 KiB tail of S2;
- M4 drains at most two retirements and grants at most two banks per boundary;
  available-bank and available-group counts are maintained incrementally;
- M4 publishes grants through a 16-entry SPSC ring, eliminating M0's nested
  oldest-grant record search;
- optional diagnostic aggregation is compiled out of the boundary; control,
  ownership, loss, current-depth, maximum-interval, and FIFO-margin fields stay;
- FIFO flush uses `STATUS0.FIFO_EMPTY` after the required delay, FIFO occupancy
  has an ambiguity-safe high-water mark, and overflow immediately poisons and
  terminates the transport epoch rather than publishing uncertain-phase data;
- USB System Error interrupt handling and PLL1 AUTOBLOCK follow the later
  manual revisions.

The stream contract is version 9. The sample stream and public API are
unchanged.

## Objective

> The only loss is a whole bank, discarded deliberately. Loss anywhere else is a
> defect.

The ring reached that for USB congestion. It has not reached it for the HSADC
FIFO. Everything below is about closing that gap by minimising GPDMA delay.

## Why the remaining work is GPDMA delay, not buffering

A bank is 16384 bytes, 8192 real samples. The host recovers I/Q by mixing at
Fs/4, a sequence of period four, and 8192 is divisible by four. **A discarded
bank is phase-safe.** It costs RF time and nothing else.

An HSADC FIFO overflow is not. The ADC fails to enqueue an unknown number of
conversions before DMA sees them. DMA still completes the programmed byte count
at the right boundary, so transport alignment survives while acquisition phase
does not. Three times in four the residue is nonzero; half of those swap I and Q
and mirror the spectrum. The count cannot be recovered afterwards: adding a known
even quantity does not change an unknown parity, flushing discards a residue of
unknown occupancy, and completion timing resolves to tens of samples where
single-sample accuracy is needed.

So the FIFO is the one place the objective can still be violated, and it is
defended by a very small amount of time.

The size of that defence is now measured, from UM10503 Rev 2.4 chapter 48
(ADCHS, added Rev 1.7, corrected Rev 1.8):

```text
FIFO depth                     16 words          (Table 1124: "up to 16 words")
PACKED_READ = 1 in FIFO_CFG    2 samples/word    => 32 samples = 1.60 us
FIFO_LEVEL = 8 raises DMA_Read_Req at 8 words    => 16 samples = 0.80 us
headroom from DMA request to overflow            => 16 samples = 0.80 us
```

**The budget for a GPDMA service delay is 0.8 microseconds.** Boundary work is
judged against that, not against the 409.6 us bank period. The quantity that
matters is the longest single stall a change can impose on a slave port GPDMA
needs, not its total cycle count.

## The metric to steer by

Overflow count is post-mortem. Add and watch instead:

- maximum observed `FIFO_STS.LEVEL` against the 16-word depth;
- longest DMA completion interval expressed as sample overrun against nominal.

Both are margin. Every change below is accepted or rejected on whether it moves
them.

`FIFO_STS.LEVEL` is bits 3:0 and **0 means empty or exactly 16 words**, not
empty. Disambiguate with `STATUS0.FIFO_EMPTY`: 1 is truly empty, 0 with LEVEL 0
means full. A high-water mark that misses this reads its most dangerous sample
as its safest.

## Work

### 1. Placement, attempted first because it is free at run time

Non-bank traffic is not distributed like the banks:

```text
S1  0x1000 0000  banks 1,3,5,7,9  + ram_usb_dma_metadata @0x1001C000 (dTDs)
S2  0x1008 0000  banks 2,6,8
S3  0x2000 0000  bank 0 + m4_share + m0_share + usb_queue_heads + M0 stack
S4  0x2000 8000  bank 4
```

The two ports carrying every shared structure also carry six of the ten banks.
S1 holds five capture banks and the dTD descriptors USB0 fetches continuously.
S3 holds the dQH, both shared-contract regions that M4 and M0 poll, M0's stack,
and a bank. Every contract access from the boundary path is an AHB transaction
on the same port as bank 0 and the queue heads.

Attempt, in order of expected value:

1. Move one bank from S1 to S2, giving 4/4/1/1. S2 is 72 KiB holding 48 KiB.
2. Move `ram_usb_dma_metadata` off S1 if any qualified region can take it.
3. Move the polled shared-contract regions off S3, or move bank 0 off S3.

This may not be achievable. The constraints are real: `0x2000C000` is shared with
the ETB, `0x18000000` is unqualified for USB0 reach, and M0 subsystem SRAM at
`0x10400000` is reached through a bridge and is a poor streaming target. Partial
success is acceptable and should be reported as which of the three landed.

`firmware.steering_model` re-proves alternation and the reserve rule against the
new address table automatically, so a placement change costs no new test work.

### 2. Boundary path

Four operations are irreducible. They cost roughly six to ten bus transactions
and cannot be moved anywhere:

```text
acknowledge hardware
publish the completed bank
choose one destination and write two descriptor words
signal M0
```

Everything else in `dma_isr` is discretionary. The current body makes **51
volatile contract accesses, of which about 37 are diagnostic aggregation** —
counters, maxima, histograms, cycle timing. A few of those carry control meaning
and stay; the bulk does not. The realistic target is **51 down to about 10**, a
five-fold cut in boundary bus traffic.

V6c already took the largest single step by replacing ownership-record scans
with masks: the ISR body now touches `buffers[]` three times, against ten records
times several volatile fields per boundary in V5c. That is banked. What remains:

- **remove diagnostic aggregation** from the boundary, behind a compile flag once
  qualified. Largest block, pure deletion, no semantics change;
- **bound the retirement drain**, currently `while (read != write)` up to sixteen.
  Masks lagging one boundary only understates available banks, which is safe;
- **maintain depth and group counts incrementally** instead of calling
  `adc_ring_bit_count` and `adc_ring_group_count` every boundary;
- **cap grants at two per boundary**, two being the minimum that lets a backlog
  contract by one bank per boundary.

**Grants can be bounded but not relocated.** They live in the ISR deliberately,
to close the M4-main-versus-M4-ISR race that stranded a bank by committing a
grant after the ISR had already steered DMA into it. Moving them to the idle path
reintroduces that race unless interrupts are masked around the commit, which puts
an interrupt-disable window exactly at the bank boundary. Against a 0.8 us budget
that trade is not worth making.

Ranked by value per unit of risk: diagnostic removal, then placement, then
bounding, then grant capping. Grant capping is last because it touches the
mechanism most recently fixed.

Two distinct levers act on the same 0.8 us. Reducing boundary work attacks the
transaction *count*. Placement attacks whether each transaction *stalls*. One
transaction queued behind an eight-beat GPDMA burst on a contended slave costs
more than ten that are not, so the two are worth pursuing independently.

### 3. M0 submit path, which has not had the V6c treatment

`adc_stream_submit_ready` still selects the oldest granted bank by scanning every
record: an outer loop over ten banks wrapping an inner ten-bank scan that reads
`granted_generation`, `produced_generation` and `submitted_generation` from each
record. Up to a hundred iterations and three hundred volatile reads per call,
around sixty in the common single-submit case, on every `sev` — which M4 raises
at least once per bank boundary.

Those reads land on the same slave port as bank 0 and the USB queue heads. This
is the exact pattern V6c removed from M4 and never applied to M0, and by volume
it is now larger than anything left in the ISR: three record touches on M4
against up to three hundred reads on M0.

The fix is symmetric with a mechanism already proven here. V6c added an M0-to-M4
retirement notification ring; add the mirror, an M4-to-M0 grant ring. M4 already
owns the grant decision and the generation ordering, so it can publish the bank
index directly and M0 pops an index instead of rediscovering it. The
`granted_generation` field remains the ownership authority; the ring only removes
the search.

### 4. Already tight, checked

Recorded so these are not re-examined: no stray divisions or modulo on the
streaming paths, `adc_ring_next_index` is compare-and-wrap and the only `%` is by
sixteen, which the compiler reduces to a mask; `allocate_transfer`,
`free_transfer` and `usb_queue_transfer_complete` are O(1) list operations; the
`do/while (aborted)` loops in `usb_queue.c` are the hardware-mandated ATDTW
retry procedure and cannot be removed.

### 5. FIFO: instrument precisely, then fail hard

`adchs.c` disables every HSADC interrupt (`CLR_EN0 = STATUS0_CLEAR_MASK`, no
`SET_EN0` anywhere) and polls the sticky `STAT0_FIFO_OVERFLOW` once per boundary.
That gives the weakest signal available:

- the counter reports **bank periods containing at least one overflow**, not lost
  samples, so the true loss is unbounded and not derivable from it;
- detection lags up to a bank period, so the contaminated bank has already
  completed before the flag is seen.

Enable `SET_EN0 = STAT0_FIFO_OVERFLOW` for immediate detection, and sample
`FIFO_STS` for the occupancy high-water mark.

Two header corrections while there. `STAT0_FIFO_FULL (0x1<<0)` is a misnomer:
bit 0 is `FIFO_LEVEL_TRIG`. `FIFO_FULL (0x1<<4)` refers to a bit that Rev 1.8
made Reserved when it narrowed `LEVEL` to four bits; it is a Rev 1.7 artifact.
Neither is used, but both mislead.

**Fix the flush before relying on the epoch restart.** `adchs.c:325` does:

```c
LPC_ADCHS->FLUSH = 1;
for (i = 0; i < 5; i++) { while (LPC_ADCHS->FIFO_STS); }
```

`FIFO_STS == 0` is empty *or* 16 words, so this can exit on a completely full
FIFO. The manual also requires at least one CPU cycle between a flush and a fill
level read, so the first read can return stale zero and the whole construct
becomes a no-op. Either fault leaves residue in the FIFO at epoch start, which
begins the new epoch on the wrong phase and defeats the reason for terminating
the old one. Insert the cycle, then loop on `STATUS0.FIFO_EMPTY`.

Then, on the first overflow, one shot:

1. halt channel 0 and stop the ADC trigger;
2. publish no bank captured at or after the fault;
3. mark the stream poisoned and signal M0;
4. M0 terminates the bulk-IN epoch so the driver's existing corrupt-transfer path
   fires;
5. leave the receiver stopped until a fresh `RECEIVER_MODE_RX` rebuilds ADC, DMA,
   FIFO, generations and bank state as a new epoch.

Nothing is recovered in place. The interval that produced the overflow is not one
in which recovery logic can be assumed to run promptly.

The poison state is control flow, not telemetry, and must not be conditional on
diagnostics being compiled in.

**Driver coupling to record on the other side:** termination works because
`airspy.c:489` treats any short or non-`COMPLETED` bulk transfer as fatal. A short
bulk-IN transfer is a device-initiated epoch termination and must remain fatal.
The deliberate short or zero-length terminal transfer carries no data and signals
termination only, which is why it does not violate the rule against new framing.

### 6. Two corrections the manual acquired after this code was written

Both are small, both are documented requirements the original sources predate.

**USB system error is unhandled.** Rev 1.9 (February 2015) changed bit 4 of
`USBSTS_D` from Reserved to `SEI`, bit 4 of `USBINTR_D` to `SEE`, and added
section 25.11. The USB controller is an AHB bus master; on a bus error it sets
System Error, sets HChalted, and **clears Run/Stop by itself**, after which
software must reset the controller via HCReset before re-initialising. The
manual names the likely device-mode cause as corrupted `dTD`/`dQH` pointer
fields.

`usb_bus_event` checks `UEI`, `URI` and `PCI` only. A system error therefore
presents as the endpoint silently ceasing, with no counter and no recovery,
which is precisely the shape of failure that is hard to attribute under bus
stress. Enable `SEE`, count `SEI`, and treat it as a hard fault: it is a
controller halt, not congestion.

**`PLL1_CTRL` AUTOBLOCK is never set.** Rev 2.3 (2017) added note [1] to
Table 137: "When the PLL1 is enabled, set the AUTOBLOCK bit in the PLL1_CTRL
register to 1. This bit re-synchronizes the clock output during frequency
changes that prevents glitches when switching clock frequencies."

`cpu_clock_pll1_high_speed` and `cpu_clock_pll1_low_speed` reconfigure PLL1
while enabled, once at every stream start and once at every stop. AUTOBLOCK is
set on the `BASE_*_CLK` registers but not on `PLL1_CTRL`. The two-stage ramp
with the 50 us dwell is already correct and cites Rev 1.8 Figure 30; only this
step, added three years later, is missing. It is a one-line change.

## Invariants

- Consecutive GPDMA destinations never share an AHB slave port.
- At least two slave groups always hold a non-USB-owned bank.
- USB never references a bank inside the steering window.
- DMA never enters a bank owned by a live or stale dTD.
- Descriptor writes target only in-memory LLIs two or more hops ahead; the
  channel LLI register is never written while the channel is enabled.
- A stale completion releases only its recorded generation.
- A discontinuity is one consecutive run, and is preferred to torn, reordered or
  silently aliased data.
- Ambiguous ownership is a safe-stop fault; ordinary congestion is not.

Alternation is proven dynamically now rather than by link-time address ordering,
so it needs the model check and a must-be-zero firmware counter.

## Not doing

- No increase beyond ten banks. Twelve buys 819 us and costs ETB, unqualified
  regions, or bridge-reached memory.
- No new framing, sequence numbers, timestamps or epoch markers in the stream.
- No recovery in place after an overflow.
- No transfer-geometry or driver-policy changes.
- No new test apparatus beyond the existing model test.

## Observations behind these calls

- Two radios on one USB 2.0 bus demand 64 MB/s against a 53.2 MB/s ceiling. The
  backpressure and whole-bank discards observed there are arithmetic and correct
  behaviour, not a defect.
- In that run, ownership-protection halts, no-candidate steering faults, SRAM
  alternation violations and GPDMA errors were all zero while the FIFO
  overflowed. The channel ran correctly and the ADC outran it, which leaves bus
  arbitration as the cause.
- V5/V5b placed two consecutive destinations in local SRAM1 and produced one
  deterministic overflow per revolution; alternating removed it. UM10503 3.6
  explains it as round-robin arbitration between masters on one slave port, with
  GPDMA bursting up to eight beats against a CPU burst of one. It is arbitration,
  not errata: ES_LPC43x0 Rev 7.2 contains no GPDMA erratum.
- The commented `ram_ahb1_0` and `ram_ahb1_1` entries in the linker script show
  vanilla split its two banks across `0x20004000` and `0x20008000` deliberately
  for the same reason.
- Vanilla has no backpressure. It relies on the host draining faster than DMA
  wraps, and its minimal path is prevention through narrow timing margins rather
  than robustness. Under the same two-radio load it would skip and tear silently,
  with its overflow counter compiled out.
- ADCHS reached the user manual only in Rev 1.7 (October 2013) and was corrected
  in Rev 1.8 (January 2014), which narrowed the FIFO level fields to four bits
  and added the explanation of the LEVEL semantics. Nothing ADCHS-related
  changed after that through Rev 2.5. Firmware written against Rev 1.7 therefore
  predates both corrections, and `adchs.h` still carries a Rev 1.7 artifact.
- The full revision sweep from 1.7 to 2.5 (September 2019) was done. Only two
  further changes touch this firmware, both above: USB system error in Rev 1.9
  and the PLL1 AUTOBLOCK note in Rev 2.3. Recorded so the sweep is not repeated:
  Rev 1.9's relabelling of `0x1008A000`-`0x10092000` in Figure 8 does **not**
  split the 72 kB block, which the 2019 AHB matrix diagram still shows as one
  matrix slave, so the S2 grouping stands; Rev 2.4's new section 51.8 is debug
  memory re-mapping and an FPB attack surface, not applicable; Rev 2.5 changed
  only ISP/IAP flash-erase warnings.
- The waterfall looked usable across thousands of flagged overflow periods, which
  is not consistent with thousands of phase inversions. Either the flag can
  assert without loss, the losses were multiples of four, or the display did not
  show it. Worth one off-centre-tone capture before the poison is enforced,
  because fail-hard turns the two-radio case from degraded into stopped.
