# Firmware next advancement

Status: implemented as experimental V6; hardware qualification in progress  
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

> Absorb congestion losslessly while free banks remain. Never halt ADC because
> USB is late; overwrite the oldest retained history instead, and keep the
> retained window the freshest consecutive run.

The halt also has a second cost that is easy to miss. Under a sustained rate
shortage the protection test does not fire once; it fires at every bank
boundary, throttling DMA to the drain rate through thousands of very short
halts. The RF time lost is the same either way, but it arrives as thousands of
scattered micro-discontinuities rather than one consecutive gap. Scattered loss
is the worst shape for any downstream decoder. A single consecutive run is the
best available, and it is what this design produces.

## Fixed timing facts

At nominal 10 MS/s complex output, the ADC produces 20 million real samples/s
in 16-bit containers:

```text
40 MB/s unpacked
16 KiB bank = 409.6 microseconds
8 KiB LLI packet = 204.8 microseconds
ten banks = 4.096 milliseconds nominal
usable backlog depth = eight banks = 3.2768 milliseconds
```

Usable depth is eight, not ten, because one bank is being filled and one must be
free for the steering decision. That `n - 2` figure is the device's real
robustness number and matches the observed 649,470-cycle service gap, which is
about 7.9 bank intervals against a measured 81,883 to 82,798 cycles per bank.

This device-side latency is small relative to the established host transfer and
application queues. It is useful USB-scheduling runway, not permission for
unbounded stale data.

## Chosen architecture

V5c already computes the ownership of the bank two ahead at every bank boundary
and uses it to decide whether to stop. The same computation, at the same
instant, decides instead where to go next.

The ten qualified banks stay in one pool. There is no static partition, no
separate private ring, and therefore no mode to enter or leave.

```text
ten banks, one pool
M4-only USB grants and a two-bank DMA steering window
steering decision taken at every completed-bank boundary
```

A bank is FREE, READY, GRANTED, or SUBMITTED. M4 is the sole writer of the
grant field; M0 may attach a dTD only after observing that grant. This closes
the otherwise unavoidable cross-core race between M0 claiming a READY bank and
M4 steering DMA into it.

The implementation examines the fixed ten-entry table rather than maintaining
mutable cross-core free lists. This is bounded work, keeps ownership legible,
and avoids another shared list whose retirement updates would themselves need
synchronization. The generated image records the worst steering cycles so this
choice is qualified against the 8 KiB packet margin on hardware.

The ten banks presently occupy four distinct AHB slave ports, not two:

```text
S1  0x1000 0000  128 kB local SRAM   banks 1,3,5,7,9   5 banks
S2  0x1008 0000   72 kB local SRAM   banks 2,6,8       3 banks
S3  0x2000 0000   32 kB AHB SRAM     bank  0           1 bank
S4  0x2000 8000   16 kB AHB SRAM     bank  4           1 bank
```

The familiar "five local-SRAM1 and five other-slave" description is correct in
aggregate, but the five others are spread across three separate slaves. This
matters for steering: the constraint is that consecutive destinations differ in
slave *port*, so S2 to S3 is legal even though neither is S1. Steering therefore
has more freedom than a two-way alternation would allow, and the group sizes are
uneven (5/3/1/1) rather than balanced.

The alternating-slave rule is preserved and promoted to a named invariant.
V5/V5b demonstrated one deterministic ADC FIFO event per ten-bank revolution
when two consecutive DMA destinations used local SRAM1. V5c removed it by
alternating local-SRAM1 banks with destinations on other SRAM slaves. UM10503
3.6 explains it: masters sharing one AHB slave port arbitrate round-robin, and
GPDMA bursts up to eight beats against a CPU burst of one. It is arbitration,
not errata; ES_LPC43x0 Rev 7.2 contains no GPDMA erratum. The required property
is alternation, not the number five, and the group sizes need not be equal.

## Steering

At each completed-bank boundary, rewrite the destination of the bank two ahead:

```text
target = bank two ahead of the one just completed
prev   = slave group already committed for the bank between them
pick   = choose the oldest reusable bank from any group != prev
write  = destination words of target's two LLI descriptors
```

`prev` is known because it was chosen one boundary earlier, so alternation is
enforced against the committed sequence rather than a prediction.

Retired FREE banks are preferred. At the congestion floor, the oldest
ungranted READY bank is deliberately reused. That is what makes the retained
window the freshest contiguous run: when history must be lost, the loss is one
consecutive interval rather than many scattered ones.

Two facts from UM10503 make this safe and bound its cost. Channel SRCADDR,
DESTADDR, CLLI and CONTROL are updated "by following the linked list when a
complete packet of data has been transferred" (19.6.16, 19.6.19): the
controller holds only the current LLI and fetches the next at packet
completion, without prefetching during a packet. A descriptor two or more hops
ahead is therefore not under the controller's eye. Margin is one full packet:

```text
8 KiB packet at 10 MSPS = 204.8 microseconds
8 KiB packet at  6 MSPS = 341.3 microseconds
```

against a decision costing one list pop and two stores.

`TRANSFERSIZE` is twelve bits, maximum 4095 transfers, four bytes short of a
16 KiB bank at 32-bit width (Table 290). Two descriptors per bank is the
minimum legal encoding, not a choice, and steering therefore rewrites two
destination words per bank.

The prohibition in 19.6.18 — "Programming this register when the DMA channel is
enabled may have unpredictable side effects" — applies to the channel register
at `0x4000 2108`, not to the descriptor array in memory. Nothing may write
`C0LLI` while the channel is enabled. This distinction carries the whole safety
argument and belongs in a comment at the write site.

## The rule that removes the halt

> Never grant a bank to USB if that would leave fewer than one non-USB-owned
> bank in each of at least two distinct slave groups.

Two groups, because alternation needs a legal alternative at every boundary;
one bank each, because that is the floor at which the steering pop can still
always succeed.

This single rule makes "no free bank to steer into" unrepresentable. The GPDMA
never halts and the ADC never stops for host congestion — not by policy, but
arithmetically. Usable transport depth is eight of ten banks, the same as the
partitioned design, except the reserve floats instead of being fixed to two
addresses.

## Normal operation

1. GPDMA captures continuously; every boundary steers the bank two ahead.
2. M4 grants completed banks oldest-first subject to the reserve rule; M0
   attaches them to the linked bulk-IN dTD queue in generation order.
3. Short host congestion accumulates valid, ordered banks and the free set
   shrinks.
4. USB retirement releases the exact generation of each bank and returns it to
   its group's free list.
5. Sustained congestion shrinks the free set continuously to its floor of two.
   There is no cliff between eight and two, and no transition.

Normal operation must not add packing, copying, framing, or a new public
request. The raw byte stream remains compatible with existing libairspy.

## Degradation and recovery

Once the free set reaches its floor, completing a bank overwrites the oldest
retained history. Capture-generation and discarded-bank counters continue
advancing. Tuner, clocks, ADC configuration, and requested sample rate remain
unchanged. USB recovery may fail, retry, or escalate without stopping capture.

Recovery needs no detection logic. As dTDs retire, banks return to the free
lists, the reserve rule relaxes on its own, and depth grows back. The host's
return is observed implicitly by dTDs completing; there is nothing to detect,
no epoch to start, and no chain to switch back.

At most eight banks can be queued, so the stale queue a returning host must
drain is bounded at 3.3 milliseconds at 10 MSPS. That does not justify a flush
path.

This is controlled loss of history, not USB-owned memory corruption. A bank may
not acquire a dTD while it is inside the steering window.

## USB controller policy

There is one controller state:

```text
always:
    linked ACTIVE bulk-IN queue
```

The reserve rule means the device is never obliged to stop appending in order to
protect capture, so the severe-congestion and return states disappear along with
the mode machine. Append when the reserve rule permits; otherwise wait. Nothing
else changes.

Do not reset, flush, prime, clear halt, or reset DATA0/DATA1 while trying to
recover ordinary congestion. Application pause/resume preserves the shared data
toggle. USB bus reset and endpoint-halt recovery remain distinct USB protocol
boundaries.

Failure to retire a dTD pins the affected bank. Pinned banks reduce the free set
and are thereby already accounted for by the reserve rule; they must not stop the
ADC.

Host return needs no detection. Retiring dTDs return banks to the free lists, the
reserve rule relaxes by itself, and submission resumes. The endpoint-NAK
experiment and the probe-transfer fallback are dropped. No ZLP, marker, header,
or new stream framing is introduced.

## Ownership and ordering invariants

- Consecutive GPDMA destinations never share an AHB slave port.
- At least two slave groups always hold a non-USB-owned bank.
- USB never references a bank inside the steering window.
- DMA never enters a bank owned by a live or stale dTD.
- A stale completion may release only its recorded old generation.
- Descriptor writes target only in-memory LLIs two or more hops ahead; the
  channel LLI register is never written while the channel is enabled.
- Ambiguous ownership remains a safe-stop/reset class fault; ordinary host
  congestion does not.
- A discontinuity is preferable to torn, reordered, or silently aliased data.
- A discontinuity should be one consecutive run rather than several.

The first two invariants need attention they did not need before. V5c satisfies
alternation by static address ordering, which a link-time assertion proves.
Steering satisfies it dynamically, so that assertion no longer proves anything.

The replacement proof is cheap. Steering consults only group-level information,
so the per-group non-submitted counts are a sound abstraction of the concrete
bank states. That abstract space is 238 reachable states rather than the tens of
millions of concrete ones, which makes this an ordinary unit test rather than a
model-checking exercise. `analysis/congestion/steering_proof.py` enumerates it.

The counting argument it confirms: steering needs a non-submitted bank in a group
other than the one already committed for the next bank. If the non-submitted set
spans at least two groups, the committed group can match at most one of them, so
at least one other group has a bank available. The bank currently being filled
sits in the committed group, which is the excluded one, so it can never be chosen
by mistake.

Exhaustive enumeration shows the reserve rule is not merely sufficient but
**necessary**: without it, ten stuck states are reachable. They are worth reading
because they are counter-intuitive.

```text
non-submitted [S1=5, S2=0, S3=0, S4=0], next bank committed to S1
    -> no legal steering target
```

Five banks free, half the ring, and no legal move. The failure is not scarcity of
free memory but loss of *slave diversity*. Any reserve rule phrased as "keep N
banks free" fails to prevent this at any N. The rule has to be about how the free
banks are distributed across slave ports, which is why it is phrased that way.

The same enumeration independently reproduces the eight-of-ten depth figure: the
deepest reachable submission under the rule is exactly eight banks.

Firmware still carries a must-be-zero alternation counter, because the proof
covers the design and not its implementation.

## Telemetry additions

Extend the existing private request `0x87`, versioning the structure if its
layout changes. The transition and reentry counters are moot; there are no
transitions. Add:

- slave-alternation violations, which must be identically zero;
- free-bank count distribution, being the floating reserve depth over time;
- minimum observed free-bank count;
- steering pops that had to skip a group to satisfy alternation;
- banks deliberately overwritten because the free set was at its floor;
- consecutive-run length distribution for discontinuities;
- time spent at the reserve floor;
- stale-epoch completions;
- maximum capture-to-submit age;
- ADC FIFO overflow, which under this design should be identically zero and
  therefore remains a hard failure.

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

1. Express the single ten-bank pool, per-slave availability, and allowed
   ownership states in the host model.
2. Port `analysis/congestion/steering_proof.py` into the host model's test suite,
   parameterised by the actual bank/slave table rather than a hardcoded one, so
   that any future re-placement of banks re-proves the reserve rule or fails the
   build.
3. Add stale-generation tests covering every bank-boundary phase.
4. Generate the twenty-descriptor chain with linker/address assertions, and
   assert descriptor placement away from capture and USB slave ports.
5. Implement future-LLI destination rewriting without channel disable, writing
   only in-memory descriptors.
6. Replace the halt test at the completed-bank boundary with the steering
   decision.
7. Implement the reserve rule as an M4-only grant before the M0 submission
   site, removing the cross-core claim race.
8. Add telemetry, including the must-be-zero alternation counter.
9. Inspect generated assembly and measure worst-case steering cycles against the
   one-packet margin.
10. Build an experimental image only after the model and static contracts pass.

## Exit criteria

- No ADC channel halt caused by USB congestion in source or generated traces.
- The word "halted" does not appear in the host-congestion path.
- No consecutive pair of GPDMA destinations shares an AHB slave port, over the
  whole model state space and with the firmware counter at zero on hardware.
- At least two slave groups hold a free bank at all times.
- No bank is overwritten while USB-owned, and no dTD references a bank inside
  the steering window.
- Steering worst-case cycles are well inside one packet time at 10 MSPS.
- Congestion shorter than the available free depth remains lossless and ordered.
- Longer congestion produces counted discontinuities, each a single consecutive
  run, while capture generations continue monotonically.
- Depth recovers without any host-return detection, epoch change, or flush.
- A dTD that never retires pins its bank without stopping the ADC.
- R2 and Mini pass repeated congestion/recovery and start/stop cycles.
- Stock libairspy continues receiving the unchanged raw stream.
