# Recursive congestion: is the client/device loop closed?

Status: analysis, no implementation change proposed
Companion to `CONTENTION_PUSHBACK.md`
Experiments: E10, E11, E12 in `contention_sim.py`; output in `RESULTS.txt`

The first-order analysis asked where congestion piles up. This asks the
second-order question:

> Can the client provoke the device, which then provokes the client, which
> provokes the device again — congestion manufacturing congestion, recursively?

Short answer: **no, and for one specific reason that is worth naming, because
it is a single design decision that a well-intentioned rewrite would remove.**

## How to answer this properly

A cycle needs two edges. Measure each in isolation and multiply.

```text
A = client -> device gain
    ms of DEVICE disturbance (ADC loss + device discard)
    caused by X ms of purely client-side disturbance

B = device -> client gain
    ms of CLIENT disturbance (host drop + application drop)
    caused by X ms of purely transport-side disturbance

loop gain = A * B
```

If either edge is identically zero there is no cycle, and the other edge's
magnitude is irrelevant. If both are nonzero, the loop gain decides whether a
disturbance decays, sustains, or grows.

This decomposition matters because the two edges turn out to have completely
different character, and averaging them into one "congestion" story hides that.

## Result: edge A is identically zero

E11, shipping configuration:

| X ms (client stall) | A: device disturbance ms | gain A | B: client ms | gain B | loop gain |
|---:|---:|---:|---:|---:|---:|
| 25 | 0.000 | 0.000 | 0.107 | 0.004 | 0.0000 |
| 50 | 0.000 | 0.000 | 25.107 | 0.502 | 0.0000 |
| 100 | 0.000 | 0.000 | 75.107 | 0.751 | 0.0000 |
| 200 | 0.000 | 0.000 | 81.565 | 0.408 | 0.0000 |
| 400 | 0.000 | 0.000 | 81.565 | 0.204 | 0.0000 |

Edge B is real and substantial — a transport disturbance costs the client up to
0.75 ms per ms. Edge A is zero at every magnitude tested.

E10 pushes edge A much harder, with the trigger entirely client-side (a 200 ms
application-callback stall) and the client made *persistently* slower than
realtime, so its queue never drains on its own:

| resubmit policy | rho_c | ADC lost ms | halts | host drop ms | device reached |
|---|---:|---:|---:|---:|---|
| always (shipping) | 0.50 | 0.000 | 0 | 150.7 | no |
| always | 0.95 | 0.000 | 0 | 157.3 | no |
| always | 1.05 | 0.000 | 0 | 288.4 | no |
| always | 1.30 | 0.000 | 0 | 799.5 | no |

At `rho_c` = 1.30 the client is permanently 30% too slow to keep up. It sheds
799 ms of RF. The device notices nothing: zero ADC loss, zero halts, ever.

**Loop gain is zero. The cycle does not exist.** Not "is small", not "decays" —
the edge is absent, so no amount of client congestion can be converted into
device congestion, and recursive manufacture is impossible.

## Why: one line of code carries the whole property

The edge is absent because of `airspy.c:461-482`. When the consumer queue is
full the completion callback discards the newest block, counts it, and then
resubmits the transfer **unconditionally**:

```c
else
{
    device->dropped_buffers++;      /* airspy.c:477 */
}
pthread_mutex_unlock(&device->consumer_mp);

if (libusb_submit_transfer(usb_transfer) != 0)   /* airspy.c:482 */
```

The resubmission is not inside the `if`. It is not conditioned on consumer
state, queue occupancy, or drop count. It is O(1) work that always happens.

Consequently the number of transfers pending at the host controller is a
function of the transfer pool alone and **never a function of how the consumer
is doing**. Consumer state cannot be encoded into transport state, so it cannot
propagate upstream. That is the entire mechanism.

State it as an invariant, because this is the form worth testing:

> **Invariant L.** Every completed transfer is resubmitted before its callback
> returns, unconditionally on consumer-queue state. Host-pending transfer depth
> is therefore independent of consumer occupancy.

Everything in this document reduces to Invariant L holding.

## The counterfactual: what "adding real backpressure" would cost

The natural-seeming improvement is to stop discarding good data and instead let
the consumer's fullness push back — don't reap, don't resubmit, let the queue
apply pressure. That is modelled as `resubmit_policy = "block_when_full"`.
Identical trigger, identical everything else:

| resubmit policy | rho_c | ADC lost ms | halts | host drop ms | device reached |
|---|---:|---:|---:|---:|---|
| block_when_full | 0.50 | 49.30 | 1 | 0.000 | **YES** |
| block_when_full | 0.95 | 50.77 | 1 | 0.000 | **YES** |
| block_when_full | 1.05 | 180.60 | **392** | 0.000 | **YES** |
| block_when_full | 1.30 | 691.93 | **318** | 0.000 | **YES** |

Edge A becomes nonzero (E11: gain 0.246 at X=200 ms, 0.623 at X=400 ms) and the
loop closes with gain ~0.10–0.13.

Three things to notice.

**A purely client-side problem now destroys ADC data.** Nothing about the RF
path, the USB link, or the firmware changed. A slow application callback now
produces 692 ms of ADC/FIFO overflow.

**392 halts from a single 200 ms trigger.** That is not one event; it is a
sustained relaxation oscillation. The queue fills, reaping stops, the URB pool
drains, the device halts and destroys data, the gap lets the client catch up,
reaping resumes, the accumulated pool floods the queue, and it repeats. The
loop gain being below 1 means it would decay if the driving condition went away
— but at `rho_c` > 1 the driving condition never goes away, so it does not.

**The host drop counter goes to zero.** This is the trap. `dropped_buffers`
is the metric an engineer would watch to judge the change, and the change
drives it to zero while making everything materially worse. The damage moves
into `adc_fifo_overflow_count` on the far side of the USB link, where the host
never sees it and where it is easy to misattribute to cabling, hubs, or a
"flaky device".

The current policy is not crude. It is the load-bearing element.

## What it actually takes to close edge A by accident

Invariant L can also be broken without changing any code, by preventing the
event thread from running. E12 separates the two ways that could happen, and
they are not equally plausible.

**CPU bandwidth cannot do it.** The event thread's work is one pointer swap and
one resubmission per completion. At 40 MB/s and 256 KiB transfers that is 152.6
completions/s at ~50 us each: **0.76% of one core**, rising to 0.82% during a
catch-up burst. E12 confirms the consequence — with the driver squeezed to a
single core and 99% of it consumed by exogenous load, ADC loss is still exactly
zero and there are zero halts. Only host drops appear.

| cpu cores | exogenous load | ADC lost ms | halts | host drop ms |
|---:|---:|---:|---:|---:|
| 2.00 | 0.00 | 0.000 | 0 | 0.000 |
| 1.00 | 0.50 | 0.000 | 0 | 0.000 |
| 1.00 | 0.90 | 0.000 | 0 | 904.4 |
| 1.00 | 0.99 | 0.000 | 0 | 1389.4 |

**Scheduling latency can do it, but only past a sharp threshold.** With the
event thread permanently runnable but dispatched only every N ms, and no
injected disturbance at all:

| wake period ms | ADC lost ms | halts | host drop ms | reaches device |
|---:|---:|---:|---:|---|
| 50 | 0.000 | 0 | 0.0 | no |
| 90 | 0.000 | 0 | 596.4 | no |
| 100 | 0.000 | 0 | 661.9 | no |
| 105 | 5.084 | 13 | 727.5 | **yes** |
| 130 | 273.1 | 11 | 576.7 | **yes** |
| 200 | 662.6 | 7 | 367.0 | **yes** |

The threshold is 104.9 ms, exactly the host URB pool depth, as it must be.

So edge A opens only if a runnable thread needing 0.8% of a core is denied
dispatch for more than 105 milliseconds. That is not ordinary load; it is a
pathological host condition — a stopped-world pause, a non-preemptible driver
section, power-management stalls, or the process being frozen. It is worth
noting that this is a *latency* requirement, not a throughput one, which means
raising thread priority helps and adding cores does not.

## Why the loop is asymmetric

Edge B exists and edge A does not, and the reason is structural rather than
accidental.

Edge B exists because the client has finite queues that must do *something*
when a transport disturbance ends and the accumulated host pool flushes into an
8-slot consumer queue. Something has to give, and blocks are dropped. The client
is where the pipeline's slack runs out.

Edge A does not exist because the driver refuses to convert queue state into
transport state. The consumer queue is a purely lossy element: its overflow
response is discard, which is instantaneous, O(1), and requires no cooperation
from anything upstream. A lossy element cannot transmit pressure backwards,
because it has no way to represent "wait".

This is the general principle, and it is the one worth carrying into the
next-advance work:

> Backpressure and loss are alternatives, not complements. A stage that drops
> cannot push back; a stage that pushes back does not need to drop. Given an
> un-throttleable source, every stage that pushes back is merely relocating the
> eventual drop to a worse place — and the worst place is the ADC, which is the
> only stage whose loss is unrecoverable.

The firmware's next-advance rolling design is the same principle applied at the
other end: when the device runs out of somewhere to put samples, it overwrites
private history rather than halting the ADC. Both ends are choosing loss over
pushback. They are consistent, and that consistency is what keeps the loop open.

## A second-order loop that does exist, inside the device

Edge A is zero, but the device has a self-coupling worth stating separately.

Under the V5c halt policy, a halt leaves the transport ring full, and the ring
takes 33.15 ms to drain afterwards (E7). During that window the device has no
absorption capacity, so a second transport event halts immediately rather than
being absorbed. That is a memory effect — the device's own past raising its
susceptibility to its own future — not a loop with the client.

E4a shows it does not compound into growth: repeated device-reaching outages
cost a flat 0.98–1.06x the isolated cost regardless of spacing. So it is real
but bounded. The rolling design reduces the vulnerable window from 33.15 ms to
2.30 ms, which is the strongest argument for it.

## The provable statement, precisely scoped

What can be claimed honestly:

> Given Invariant L, no client-side condition — slow callback, blocked
> callback, oversubscribed CPU, or a consumer persistently slower than realtime
> — produces any device-side disturbance. The client-to-device edge gain is
> zero, so the loop gain is zero, so recursive congestion manufacture is
> impossible. Verified across client stalls from 25 to 400 ms and consumer
> utilisations from 0.5 to 1.3.

What must not be claimed. This is a structural argument backed by a fluid model
and a code read, not a machine-checked proof. It rests on four load-bearing
assumptions, and each is a real failure mode rather than a formality:

1. **Invariant L holds.** Resubmission stays unconditional. This is the whole
   property and it is one `if` away from being false.
2. **The event thread is dispatched within 104.9 ms.** Not a CPU-bandwidth
   requirement (0.8% of a core) but a scheduling-latency one.

   **This bound applies only to host-software stalls and must not be read as
   the device's general immunity.** A shortage of bytes-per-second on the wire
   is a different failure that the host pool cannot absorb at all: it reaches
   the ADC after roughly 3 ms, not 104.9 ms (`CONTENTION_PUSHBACK.md`, Fact 1b).
   That path does not involve the client, so it does not create edge A and does
   not affect the loop-gain result — but it is by far the likeliest way the
   current V5c firmware reaches the ADC in the field, and quoting 104.9 ms as
   though it covered every case would be misleading.
3. **The shared mutex is not held across the application callback.** It is not
   today — `airspy.c:371` unlocks first — but a rewrite that widened the
   critical section to cover conversion or the callback would put application
   stall duration directly onto the completion thread's critical path and
   create edge A through the lock rather than through the queue.
4. **The hard-stop paths do not fire.** `airspy.c:484` and `airspy.c:489` end
   the stream on any submit failure, short transfer, or non-`COMPLETED` status.
   This is not a loop, and it is not recursion, but it is strictly worse than
   either: gain is unbounded and the outage is permanent until the application
   restarts. Any loop analysis is moot if this triggers.

Assumptions 1 and 3 are properties of code that the rewrite will touch.
Assumption 2 is a property of the host. Assumption 4 is the one the driver's
own next-advance document already commits to fixing.

## Consequences for the planning work

**Make Invariant L an explicit, tested contract rather than an emergent
property.** It currently holds because of where one statement sits relative to
one brace. The rewrite should state it, and should have a test that fails if
host-pending depth ever becomes a function of consumer occupancy — for example,
asserting that submitted-minus-completed equals the transfer count at every
callback exit, under a synthetic consumer that never drains. The driver's exit
criterion "sixteen USB transfers remain pending independently of consumer
ownership" is already exactly this property; it deserves to be named as the
anti-recursion invariant, because that is what it is buying.

**Treat "honour consumer backpressure" as a rejected design, with the reason
recorded.** It is an attractive idea, it will be proposed again, and the
counterfactual above is the answer: it converts a client-only problem into ADC
destruction while zeroing the counter that would reveal it. The existing
non-goal "no change from dropping the newest completed host block when the
consumer queue is full" is correct; this document supplies its justification.

**Measure event-thread dispatch latency directly, not throughput.** The single
host property the whole no-loop argument depends on is a scheduling-latency
bound, and no throughput or drop-rate metric exposes it. A max-observed
dispatch-gap counter on the event thread would; anything approaching 100 ms is
the warning that edge A is about to open.

**Keep the loss domains separate in telemetry.** The counterfactual is
instructive precisely because its damage moved between domains while the
headline counter improved. ADC/FIFO overflow, device discard, host queue drop,
and application drop must stay individually visible or this class of regression
is invisible.

## Model limits specific to this analysis

The loop analysis inherits every limit in `CONTENTION_PUSHBACK.md`. Three
additional ones matter here.

The CPU model is fair-share between two runnable threads with an exogenous load
term. It does not model priority inversion on `consumer_mp`, which is the one
mechanism that could create edge A without violating Invariant L or the 105 ms
dispatch bound. That path is argued from the code (assumption 3) and is not
simulated; the critical sections are short enough that a fluid model would not
resolve it. Instrumenting real lock-hold and lock-wait times is the right way to
close that gap.

`block_when_full` is a deliberately simple counterfactual: it refuses to reap
while the queue is full. A real backpressure implementation might be gentler —
watermarks, partial reaping, or pacing. Those would reduce edge A's gain without
removing it. The qualitative conclusion (edge A becomes nonzero, damage moves to
the ADC, the host drop counter stops reporting it) does not depend on how
gentle the mechanism is.

The 104.9 ms threshold is exactly the URB pool depth and therefore scales with
transfer geometry. Any future tuning that reduces total outstanding host bytes
reduces the scheduling-latency margin proportionally. That is a specific and
under-appreciated cost of shrinking the transfer pool, and it should be part of
the tuning work item's evaluation rather than discovered afterwards.
