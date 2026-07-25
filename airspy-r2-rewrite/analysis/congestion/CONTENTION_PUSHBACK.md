# Contention pushback: where congestion piles up, and where it multiplies

Status: analysis, no implementation change proposed
Scope: Airspy One (R2/Mini) capture chain, ADC through application
Method: code trace plus a fluid discrete-event model, `contention_sim.py`
Results: `RESULTS.txt` (regenerate with `python3 contention_sim.py`)

This answers one question: starting at the application and walking backwards to
the ADC, where does congestion accumulate, and can congestion at one stage
*create* congestion at another rather than merely passing it along.

The working definition throughout:

> Congestion **multiplies** when a disturbance of magnitude T produces total
> system disturbance greater than T, or when two disturbances that are
> individually harmless become harmful in combination.

Multiplication is the sin. Loss by itself is not; with an un-throttleable source
some loss is arithmetic, not a defect.

## The chain, stated backwards

```text
application (SDR#)          consumes callback blocks, own ring + DSP thread
  libairspy consumer thread unpack -> convert -> IQ half-band -> app callback
  consumer queue            8 x 256 KiB, drop-newest          airspy.c:461-478
  libusb event thread       reap, pointer-swap, resubmit      airspy.c:447-491
  host URB pool             16 x 256 KiB async bulk-IN        airspy.c:882-883
--- USB 2.0 HS bulk, endpoint 0x81, 512 B packets ---
  dTD queue                 pool = 10                    usb_endpoint.c:56-62
  M0 submit                 in-order, per completed bank  airspy_m0.c:300-338
  SRAM bank ring            10 x 16 KiB              stream_contract.h:9-10
  GPDMA channel 0           halt policy              airspy_m4.c:693-706
  ADCHS FIFO                small, overflows in microseconds
  ADC                       20 MS/s real, 16-bit containers, un-throttleable
```

Two rates govern everything. The source is `lambda` = 40 MB/s unpacked
(30 MB/s with 12-bit packing). The link drains at `mu`, the *effective* bulk-IN
payload rate, which on ordinary COTS hosts sits somewhere between 40 and 48 MB/s
against a 53.248 MB/s theoretical ceiling. Write `rho = lambda/mu`.

## Fact 0: the source cannot be pushed back

Every other stage in this system can be told to wait. The ADC cannot. It is
clocked, it is free-running, and there is no credit, window, pause, or flow
control that reaches it. Therefore **backpressure in this system never
terminates in a slowdown; it always terminates in a drop.** The only engineering
choices available are *where* the drop happens, *which* domain absorbs it, and
*how fast* the system becomes able to absorb the next one.

This is why "congestion" borrowed from networking is only half-applicable. There
is no source to throttle, so there is no stable congestion-avoidance equilibrium
to reach. There is only a loss location policy.

## Fact 1: the elasticity ladder is overwhelmingly host-side

Measured directly from the pinned geometry (`RESULTS.txt`, E0):

| stage | bytes | ms of RF time |
|---|---:|---:|
| ADCHS FIFO | 512 | 0.013 |
| SRAM bank ring | 163,840 | 4.096 |
| dTD queue (aliases the ring) | 163,840 | 4.096 |
| host URB pool | 4,194,304 | 104.858 |
| consumer queue | 2,097,152 | 52.429 |
| **total** | | **~161.4** |

The device holds 4.096 ms. The host holds 157.3 ms. The ratio is about 1:38.

The consequence is counter-intuitive and it is the single most useful planning
fact in this document: **for host-side disturbances, the device ring is almost
irrelevant, because the host's own 104.9 ms URB pool absorbs the event long
before the device notices.** E8 measures the marginal value of a bank directly:

| banks | ring ms | longest lossless host stall | marginal ms per bank |
|---:|---:|---:|---:|
| 4 | 1.638 | 101.36 | — |
| 8 | 3.277 | 103.11 | 0.375 |
| 10 | 4.096 | 103.86 | 0.375 |
| 16 | 6.554 | 106.36 | 0.375 |
| 20 | 8.192 | 108.11 | 0.438 |

Each added bank buys one bank time, ~0.41 ms, on top of a 104.9 ms host pool.
Doubling the ring from 10 to 20 banks extends survivable host outage by 4%.

**This conclusion is correct only for host-software stalls, and stating it
without that qualifier was an error. See Fact 1b, which is the more important
case.**

### Fact 1b: the host pool does not protect against a slow wire

The 104.9 ms figure describes one failure only: the host stops *reaping and
resubmitting* while the link still runs. It says nothing about the link itself
being slow, and the two are completely different.

If effective bulk rate dips below the ADC's 40 MB/s — a competing device, a hub
transaction translator, a periodic/isochronous reservation, an error-and-retry
burst, or the controller servicing another endpoint — the device ring fills at
`(lambda - mu_eff)` **no matter how many URBs are queued**. Host buffer space is
irrelevant when the shortage is bytes-per-second on the wire rather than
somewhere to put them. The device ring is then the entire defence.

E16 measures it. A link-side interruption, with the full 104.9 ms host pool
available throughout:

| wire stall ms | halt policy: ADC lost ms | halts | rolling: device discard ms | rolling ADC lost |
|---:|---:|---:|---:|---:|
| 2 | 0.000 | 0 | 2.235 | 0.000 |
| 3 | 0.093 | 1 | 3.235 | 0.000 |
| 4 | 1.093 | 1 | 4.235 | 0.000 |
| 8 | 5.093 | 1 | 8.235 | 0.000 |
| 20 | 17.093 | 1 | 20.235 | 0.000 |

**A wire stall of roughly 3 ms halts the ADC. The equivalent host-software
stall needs more than 104.9 ms.** Those two thresholds differ by a factor of 35,
and only one of them describes a rare event. This is the mechanism by which
V5c reaches the ADC in the field: not a pathological host, just a few
milliseconds of bulk starvation.

And against *this* disturbance, ring depth is worth its full face value, 1:1:

| banks | ring ms | longest lossless wire stall ms |
|---:|---:|---:|
| 4 | 1.638 | 0.25 |
| 8 | 3.277 | 2.00 |
| 10 | 4.096 | 2.75 |
| 16 | 6.554 | 5.25 |
| 24 | 9.830 | 8.50 |

The slope is 0.4125 ms per bank — exactly one bank time. So the honest summary
is: **device ring depth buys almost nothing against host-software stalls and
buys full value against wire stalls, and the wire stall is the one that
actually happens.**

Note also that the ten-bank ring yields only 2.75 ms rather than its nominal
4.096 ms. The halt test at `airspy_m4.c:693-706` protects the bank *two* ahead,
so it trips with two banks still unused — about 0.82 ms of the budget, a fifth
of the ring, spent on boundary margin. Whether that margin needs to be two banks
is a measurable question and worth asking, since it is the cheapest available
improvement to the threshold that matters.

A useful field diagnostic falls out of this: **ADC loss with zero host drops is
a wire/rate problem; host drops with zero ADC loss is a slow consumer.** The two
are distinguishable from counters alone, and they have nothing to do with each
other.

### Mean link rate is not the criterion; episode deficit is

E15 makes the point that matters for planning. Effective rate drops to 30 MB/s
for D ms out of every 200 ms, so the *mean* stays above 40 MB/s throughout:

| episode ms | duty | mean mu MB/s | deficit per episode | halt: ADC lost ms | rolling: ADC lost |
|---:|---:|---:|---:|---:|---:|
| 5 | 0.03 | 42.67 | 1.25 ms | 0.000 | 0.000 |
| 10 | 0.05 | 42.35 | 2.50 ms | 0.000 | 0.000 |
| 20 | 0.10 | **41.70** | 5.00 ms | **18.90** | 0.000 |
| 40 | 0.20 | 40.40 | 10.00 ms | 63.84 | 0.000 |
| 60 | 0.30 | 39.10 | 15.00 ms | 108.90 | 0.000 |

At 10% duty the link averages 41.7 MB/s — comfortably above the ADC — and the
firmware still destroys 18.9 ms of RF. The transition happens exactly where the
per-episode deficit crosses the ring's ~2.75 ms trip point, not anywhere near
where the mean crosses 40.

So the sizing rule is:

> The ring must absorb the worst single *episode* deficit, `(lambda - mu_low) *
> D`. Average throughput headroom does not substitute for it, and a link that
> measures fine on a throughput test can still be destroying RF.

This also means bus contention has to be characterised as episode depth and
duration, not as a mean or a percentile of achieved MB/s. A throughput
benchmark cannot detect the failure that matters here.

Rolling records zero ADC/FIFO events in every row above. It does not make the
loss free — the same RF time reappears as counted device discard — but it keeps
host-side contention out of the ADC health domain entirely, which in this
regime is the whole difference between the two policies.

## Fact 2: the intended firewall exists and it works

The driver's design intent is that application slowness is converted into
counted host-block drops and never propagates upstream. That intent is
correctly implemented. Two details make it real:

The consumer thread releases the queue mutex at `airspy.c:371`, *before* it does
conversion, IQ filtering, and the application callback. A slow or blocked
application therefore does not hold the lock that the libusb completion thread
needs.

When the queue is full, `airspy.c:477` increments a drop counter and
`airspy.c:482` resubmits the transfer anyway. USB depth is preserved
unconditionally.

E2 confirms it: an application-callback stall of any duration up to 400 ms
produces **zero** ADC loss, zero device discard, zero halts, and a device ring
that never exceeds one occupied bank. Host drops appear only once the stall
exceeds the 52.4 ms consumer queue, and then track `stall - 52.4 ms` as
arithmetic requires. The firewall holds.

This vindicates the original design. The "crude" immediate-resubmit-and-count
policy is precisely the thing that stops application congestion from reaching
the ADC, and it should not be softened.

## Fact 3: congestion does not multiply at the device

E4a repeats a 120 ms host outage — long enough to exhaust the URB pool and
genuinely reach the device — five times at spacings from 125 ms to 400 ms:

| spacing ms | RF lost ms (halt) | per stall | vs isolated |
|---:|---:|---:|---:|
| 125 | 357.9 | 71.6 | 1.06 |
| 140 | 339.2 | 67.8 | 1.00 |
| 200 | 343.3 | 68.7 | 1.01 |
| 400 | 330.7 | 66.1 | 0.98 |

Isolated single-stall loss is 67.7 ms. The per-stall cost is flat at 0.98–1.06
regardless of spacing, under both the halt and rolling policies. Repeated
device-reaching outages cost their arithmetic sum and nothing more.

E7 makes the same point across policies. Under a 130 ms outage:

| policy | ADC/FIFO ms | device discard ms | host drop ms | total RF ms | torn banks | elasticity restored |
|---|---:|---:|---:|---:|---:|---:|
| halt (V5c) | 23.53 | 0.00 | 52.43 | 75.96 | 0 | 33.15 ms |
| rolling (next advance) | 0.00 | 26.68 | 52.43 | 79.10 | 0 | **2.30 ms** |
| overwrite (vanilla 2-bank) | 0.00 | 26.21 | 52.43 | 78.64 | 1 | 0.39 ms |

Total RF loss is conserved at 76–79 ms across all three. That is the correct
physics: the ADC produced ~26 ms of samples with nowhere to put them, plus
52.4 ms of host-queue overflow, and no policy can conjure storage that does not
exist. **The policies do not differ in how much they lose. They differ in which
domain absorbs the loss, and in how fast the system is ready for the next
event.**

## Fact 4: congestion multiplies at the consumer queue

This is the affirmative answer to the question. E4b holds the disturbance fixed
— five 30 ms application stalls — and varies only their spacing, at three
consumer utilisations. `rho_c` is convert+IQ+callback cost divided by the
realtime budget of one 256 KiB block.

At every utilisation tested, **an isolated 30 ms stall costs exactly zero RF
time.** It fits inside the 52.4 ms queue. Yet:

| spacing ms | RF lost, rho_c=0.5 | rho_c=0.8 | rho_c=0.9 |
|---:|---:|---:|---:|
| 40 | 58.98 | 98.30 | 98.30 |
| 50 | 19.66 | 85.20 | 91.75 |
| 60 | 0.00 | 72.09 | 91.75 |
| 80 | 0.00 | 45.88 | 85.20 |
| 120 | 0.00 | 13.11 | 58.98 |
| 200 | 0.00 | 0.00 | 26.21 |
| 300 | 0.00 | 0.00 | 0.00 |

Five events that individually cost nothing, spaced 40 ms apart, destroy 59 to
98 ms of RF. **The entire loss is created by the spacing.** That is congestion
multiplying, and it is the mechanism the planning work should target.

### The recovery shadow law

The mechanism is not mysterious. After a stall of duration T, the consumer queue
holds an extra `lambda*T` of backlog, and it drains only at the consumer's
*excess* rate. The time to return to baseline is

```text
recovery shadow = T * rho_c / (1 - rho_c)
```

A second disturbance arriving inside that shadow meets a queue that has not
recovered, so it overflows a queue that would otherwise have absorbed it.

E9 validates the law against the simulation directly, using a single 40 ms
stall and measuring how long delivered sample age stays elevated:

| rho_c | measured latency recovery ms | predicted `T*rho/(1-rho)` |
|---:|---:|---:|
| 0.30 | 14.7 | 17.1 |
| 0.50 | 34.4 | 40.0 |
| 0.70 | 80.3 | 93.3 |
| 0.80 | 147.3 | 160.0 |
| 0.90 | 337.4 | 360.0 |
| 0.95 | 711.3 | 760.0 |

Measured tracks predicted within 10% across a 40x range, running slightly under
because the queue is not completely full at the moment the stall ends.

The law predicts the E4b table without fitting anything. For a 30 ms stall the
shadow is 30 ms at `rho_c`=0.5, 120 ms at 0.8, and 270 ms at 0.9; adding the
stall itself gives interaction thresholds of 60, 150 and 300 ms. The measured
thresholds where loss first reaches zero are exactly 60, between 120 and 200,
and exactly 300. The law is predictive, not descriptive.

The practical statement: **a consumer running at 95% of realtime turns a 40 ms
hiccup into a 711 ms window of elevated latency during which the system is
fragile.** Nothing about that is visible in throughput or in drop counters
during the quiet periods.

## Fact 5: the same law governs the device, with rho set by the wire

The device ring drains at the link's excess rate, `mu - lambda`, so it obeys the
identical law with `rho = lambda/mu`. Unpacked at 40 MB/s the margin is thin:

| mu MB/s | rho | stretch `rho/(1-rho)` |
|---:|---:|---:|
| 41.0 | 0.976 | 40.0 |
| 43.0 | 0.930 | 13.3 |
| 45.0 | 0.889 | 8.0 |
| 48.0 | 0.833 | 5.0 |
| 53.2 | 0.752 | 3.0 |

With 12-bit packing enabled the source drops to 30 MB/s and the same link gives
stretch 2.3 at 43 MB/s and 1.7 at 48 MB/s.

E6 measures this end to end, stalling each configuration exactly 25 ms past its
own URB pool depth so the device is reached by an identical margin and only
`rho` differs:

| lambda | mu | rho | ADC lost ms | elasticity restored ms | analytic stretch |
|---:|---:|---:|---:|---:|---:|
| 40 | 43 | 0.930 | 25.82 | 33.15 | 13.3 |
| 40 | 45 | 0.889 | 25.80 | 20.03 | 8.0 |
| 40 | 48 | 0.833 | 25.78 | 12.63 | 5.0 |
| 30 | 43 | 0.698 | 26.77 | **8.01** | 2.3 |
| 30 | 45 | 0.667 | 26.75 | 7.29 | 2.0 |
| 30 | 48 | 0.625 | 26.73 | 5.81 | 1.7 |

The loss is the same in every row. The *vulnerability window afterwards* differs
by 4x between the worst and best row.

This reframes packing. Packing has always been argued as a throughput or
headroom measure and rejected on the cost of the in-place 8 KiB implementation.
The congestion argument is different and stronger: **packing shortens the
post-event fragile window by roughly 4x, because it moves `rho` away from 1.**
That is an argument about smoothness, not about peak rate, and it survives even
if peak rate is already adequate.

## Fact 6: recovery is a burst, and the burst is a second disturbance

Every stall becomes a gap followed by a catch-up burst. The burst is capped by
the recovering stage's maximum rate, not by the mean. E5 measures sustained
delivery at the application boundary over a 25 ms window, as a multiple of
realtime:

| stall ms | injected at | peak/realtime | app ring peak | app-level drop ms |
|---:|---|---:|---:|---:|
| 5 | event thread | 1.05 | 1 | 0.00 |
| 20 | event thread | 1.57 | 3 | 0.00 |
| 60 | event thread | 2.10 | 3 | 23.70 |
| 60 | app callback | 2.10 | 3 | 27.23 |
| 200 | app callback | 2.10 | 3 | 23.06 |

The 60 ms event-thread row is the important one. That stall was **absorbed
losslessly by the device and by the URB pool** — zero ADC loss, zero halts. It
nevertheless destroyed 23.7 ms of RF at the *application*, because the recovery
burst arrived at 2.1x realtime into a downstream ring sized for the mean.

So congestion propagates *forward* across a stage that successfully absorbed it.
A stage that suffers no loss still emits a disturbance. This is the recursive
path the planning work asked about, and it is real:

```text
stall at stage k
  -> stage k emits: gap of length T, then burst at mu_k for T*rho/(1-rho)
    -> stage k+1 must absorb lambda*T of excess or drop
      -> if it drops, it emits its own gap-then-burst
        -> ...
```

The disturbance *volume* (`lambda*T`) is conserved down the chain. Its
*duration* is stretched by `rho/(1-rho)` at each stage. Every downstream stage
is sized for the mean, so each one re-presents the same excess volume as a fresh
overflow decision.

The structural rule that follows: **for congestion not to multiply forward, each
stage's slack must be at least the excess volume the upstream stage can emit
during catch-up.** Today the last hop — libairspy consumer thread to the
application — is the one place where we neither control nor know the downstream
buffer, and it is where E5 shows the loss landing.

## Fact 7: the latency ratchet

When the consumer queue is full, `airspy.c:461-478` discards the **newest**
completed block and keeps the older ones. The queue therefore stays full, and
the standing latency stays pinned at its maximum.

E9 shows delivered sample age rising from a ~9–13 ms baseline to a 46–53 ms peak
— essentially the entire queue depth converted into latency — and then decaying
only at the consumer's excess rate. During that whole window the application is
being fed RF that is up to 52 ms old while fresher samples are being thrown away
at the door.

This is not a correctness bug and the driver documents the policy deliberately.
But it has a real cost: after any congestion event, AGC, AFC, squelch and the
user's own tuning all operate on stale RF for the length of the recovery shadow,
and at high `rho_c` that is hundreds of milliseconds. Drop-newest minimises
*disruption*; it maximises *staleness*. The next-advance discussion of firmware
rolling mode already adopts the opposite preference on the device side —
"resume USB from recent samples" — and the two ends of the pipeline currently
disagree about which they value.

Note that drop-oldest alone does not fix this; it delivers recent data but the
queue still stays full, so the standing latency is unchanged. Restoring recency
requires actually shedding the standing backlog, which is a policy question for
the planning work, not a code change proposed here.

## Fact 8: elasticity restore time is the metric that predicts multiplication

Pulling Facts 3, 4 and 5 together: total loss from an isolated event is fixed by
arithmetic and no policy improves it. What policy controls is how long the
system remains unable to absorb the *next* event. That is the quantity that
turns independent events into compounding ones.

From E7, under an identical 130 ms outage:

```text
halt (V5c)      elasticity restored 33.15 ms after the host returns
rolling         elasticity restored  2.30 ms after the host returns
```

A 14x difference, with the same RF loss. Under halt, the ring is still full when
the host comes back, so a second outage within 33 ms halts immediately. Under
rolling, the ring is empty within 2.3 ms and the next outage gets the full
budget again.

The halt policy has a second cost the totals hide. It lands its 23.5 ms in the
`adc_fifo_overflow` domain rather than the discard domain, meaning a device
health event and a `backpressure_discontinuity_count` increment for what is
purely a host problem. And because `C0CONFIG` bit 18 is set *after* DMA has
already entered the next bank (`airspy_m4.c:705`), resumption continues filling
that same bank from its current offset — so one bank carries pre-stall and
post-stall samples spliced with no marker, and is delivered to the host as a
normal full 16 KiB bank. The driver already accepts that full buffers may
contain discontinuities, so this is consistent, but it is a silent tear rather
than a counted boundary. The rolling design avoids it by switching on a future
LLI at a bank boundary.

This is the strongest simulation-backed argument for the rolling design in
`firmware/NEXT_ADVANCEMENT.md`, and notably it is *not* the argument that
document leads with. Rolling does not reduce loss. It restores elasticity 14x
faster and keeps host congestion out of the ADC health domain.

## Structural couplings the fluid model does not capture

These come from reading the code, not from the simulation. They are the paths by
which the Fact 2 firewall could fail in the field, and they matter because the
firewall is the thing preventing consumer congestion from reaching the ADC.

**The firewall is scheduling-dependent, and its failure is correlated with the
condition it protects against.** The firewall holds because the libusb event
thread keeps reaping and resubmitting while the consumer is stalled. Both are
threads on the same host. The same CPU shortage that stalls the application
callback also delays the event thread. E3 shows the device needs a >104.9 ms
event-thread outage before it loses anything, which is a large margin — but the
independence assumption behind that margin is false under real load. The two
stalls are drawn from the same cause. Any measurement campaign should sample
event-thread scheduling latency directly, not infer it from throughput.

**One mutex spans both threads with no priority inheritance.** `consumer_mp` is
taken by the completion callback at `airspy.c:459` and by the consumer at
`airspy.c:354` and `airspy.c:436`. The consumer's critical sections are short
and it correctly releases before the callback, so the exposure is preemption
*inside* a few pointer operations. But a default pthread mutex has no priority
inheritance on Linux or macOS, and the completion thread blocking there stalls
reaping for *all sixteen* transfers, not just one. The window is small and the
consequence is disproportionate.

**Three transient conditions are converted into permanent stream death.**
`airspy.c:484` sets `streaming = false` on any submit failure, and
`airspy.c:489` does the same on any non-`COMPLETED` status or any short
transfer. Gain here is unbounded: a momentary kernel resource shortage becomes
an outage that ends only when the user restarts the application. The driver's
next-advance document already commits to classifying these cases and is right to
— it is the only place in the whole chain where a transient can become
permanent. Note the transfers are filled with timeout 0 (`airspy.c:258`), so
timeout is not among the triggers.

**Host completion granularity is 16 device banks.** A 256 KiB URB completes only
when all sixteen 16 KiB banks have arrived, so up to 6.55 ms of device-side
state is invisible to the host, and a device-side stall does not complete the
in-flight URB at all — it simply starves the consumer, which then gets a burst.
Fact 6's starve-then-burst pattern is a direct consequence. The driver's
non-goal "no assumption that USB completion boundaries equal firmware bank
boundaries" is the right instinct.

**The consumer holds a queue slot for the whole callback.**
`received_buffer_count--` happens at `airspy.c:437`, after the callback returns,
so effective queue capacity is seven plus one in service. This is correct — it
prevents the completion thread from swapping into a buffer being read — but it
means the advertised 8-slot, 52.4 ms depth is 7 slots of true queueing.

## What this says for the planning work

Restating the findings as decisions, without proposing implementation:

The device ring should be sized against wire stalls, not host stalls. Against
host-software outages it buys 0.41 ms per bank on top of 104.9 ms and is nearly
pointless. Against bulk starvation on the wire it is the only defence there is
and buys full value, 1:1. Ten banks currently survive ~2.75 ms of wire stall.
That number, not the 104.9 ms one, is the device's real robustness figure, and
it is the one to quote and design against.

The two-bank lookahead in the halt test costs a fifth of the ring. Protecting
the bank two ahead rather than one spends ~0.82 ms of a 4.096 ms budget on
boundary margin. If one bank is provably sufficient, the wire-stall threshold
rises from 2.75 to ~3.2 ms at no cost.

The rolling design's justification should be elasticity restore time, not loss
reduction. It does not reduce loss — E7 shows 79.1 ms versus 75.9 ms, slightly
worse — and claiming otherwise will not survive measurement. It restores
absorption capacity 14x faster and keeps a host problem out of the ADC health
domain. Those are the defensible claims and they are strong.

Consumer utilisation `rho_c` deserves to be a first-class, measured quantity.
It is the parameter that determines whether independent hiccups compound, via
a law that predicts the observed thresholds without fitting. It is not currently
observable through any counter the driver exports. Sample age at the callback
and time-above-baseline would expose it.

Packing has a congestion argument distinct from its throughput argument: it
moves `rho` from 0.93 to 0.70 and shortens the post-event fragile window ~4x.
That argument should be evaluated on its own terms rather than inheriting the
verdict from the throughput discussion.

The forward burst is the unowned risk. A stage that absorbs a stall losslessly
still emits a 2x burst that the next stage must absorb, and the last hop hands
that burst to an application whose buffering we do not control. Any pacing or
shed-to-recency policy belongs at that boundary, and it is the one place where
the current design has no policy at all.

Loss-domain separation should be preserved and extended. ADC/FIFO overflow,
deliberate device discard, host queue drop, and application-side drop have
different meanings and different fixes. The firmware telemetry already
separates the first two; `dropped_samples` currently merges the third into the
application's view. Fact 6 shows the fourth exists and is invisible today.

## Model limits

The simulation is a fluid model with a 5 microsecond timestep; bank and URB
boundaries are exact to within one step, roughly 0.03% of a bank. It models
rates and occupancies, not USB microframe scheduling, NAK/ping behaviour, hub
transaction translation, cache and memory-bandwidth effects, interrupt latency
distributions, or scheduler preemption. `mu_usb` is a single effective payload
rate standing in for all of that.

The ADCHS FIFO is parameterised at 512 bytes; the exact depth changes only how
many microseconds of a halt are survivable, not any conclusion here, because
every halt observed is milliseconds long.

The application stage is a generic downstream consumer — a ring plus a realtime
drain — not a model of SDR#. Its source is not in this tree. Fact 6's app-level
drop numbers should be read as "a downstream consumer sized for the mean drops
this much", not as a measurement of any particular application. The `rho_c`
findings do not depend on it.

`mu_usb` defaults to 43 MB/s. Real controllers vary and no single number is
universal; E1 and E6 sweep it precisely because the conclusions are sensitive to
it. Nothing here substitutes for the Windows/macOS/Linux matrix the driver
document already requires.
