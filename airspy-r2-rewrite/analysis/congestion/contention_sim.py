#!/usr/bin/env python3
"""
Contention-pushback simulator for the Airspy One (R2/Mini) capture chain.

Traces the full transport from the un-throttleable ADC to a generic downstream
application consumer (the SDR# stand-in), and measures whether a disturbance
introduced at one stage *multiplies* into disturbances at other stages.

Stage chain modelled (device -> host -> application):

    ADC (fixed 20 MS/s real, 16-bit containers, cannot be slowed)
      -> ADCHS FIFO (tiny)
        -> GPDMA -> SRAM bank ring (N x 16 KiB)
          -> M0 dTD submission (pool P)
            -> bulk-IN endpoint 0x81 (USB 2.0 HS, effective rate mu_usb)
              -> host URB pool (K x 256 KiB, libusb async transfers)
                -> libusb event thread (reap + swap + resubmit)
                  -> libairspy consumer queue (Q x 256 KiB, drop-newest)
                    -> consumer thread (unpack + convert + IQ + app callback)
                      -> application ring (generic downstream consumer)

Everything is a fluid model over a fixed timestep with exact byte accounting at
stage boundaries. Rates are bytes/second, times are seconds internally and
reported in milliseconds.

Loss is deliberately accounted in three separate domains, because they are three
different failures with different costs:

    adc_lost      - ADC/FIFO overflow. Irreversible, device-level, and in the
                    halt policy it also implies a FIFO error event.
    device_discard- capture history intentionally overwritten on the device
                    (rolling mode). RF time is lost but the device stays healthy.
    host_drop     - a fully received 256 KiB host block discarded because the
                    consumer queue was full. Costs RF time, costs no device
                    health, and is the outcome the current driver deliberately
                    chooses.

Usage:
    python3 contention_sim.py            # run the full experiment suite
    python3 contention_sim.py --quick    # shorter sweeps
"""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

# ---------------------------------------------------------------------------
# Geometry pinned to the source in this tree
# ---------------------------------------------------------------------------
# firmware/common/stream_contract.h : AIRSPY_STREAM_BUFFER_COUNT = 10
#                                     AIRSPY_STREAM_BUFFER_BYTES = 16 * 1024
# firmware/airspy_m0/usb_endpoint.c : bulk-IN dTD pool = AIRSPY_STREAM_BUFFER_COUNT
# driver/current/libairspy/src/airspy.c : transfer_count = 16
#                                         buffer_size    = 262144
#                                         RAW_BUFFER_COUNT = 8

BANK_BYTES = 16 * 1024
URB_BYTES = 262144
ADC_RATE_UNPACKED = 40.0e6          # 20 MS/s real x 2 bytes
ADC_RATE_PACKED = 30.0e6            # 12-bit lossless packing

FREE, READY, SUBMITTED = 0, 1, 2


@dataclass
class Params:
    # --- source ---
    adc_rate: float = ADC_RATE_UNPACKED
    fifo_bytes: float = 512.0        # ADCHS FIFO; small, parameterised

    # --- device ring ---
    n_banks: int = 10
    bank_bytes: int = BANK_BYTES
    dtd_pool: int = 10
    policy: str = "halt"             # "halt" | "rolling" | "overwrite"
    # rolling-mode partition (policy == "rolling")
    transport_banks: int = 8
    private_banks: int = 2
    rolling_trigger: int = 6         # transport-ring occupancy that arms the switch
    rolling_reentry: int = 0         # transport occupancy required to re-enter

    # --- link ---
    mu_usb: float = 43.0e6           # effective bulk-IN payload rate

    # --- host transport ---
    n_urbs: int = 16
    urb_bytes: int = URB_BYTES
    t_reap: float = 50e-6            # per-completion cost on the event thread
    # M0 interrupt + usb_queue_transfer + prime, paid only when the bulk-IN
    # endpoint has no linked successor dTD. Stock firmware has pool=1 so it
    # pays this on every bank; a linked queue pays it almost never.
    m0_prime_latency: float = 0.0

    # --- host consumer ---
    consumer_slots: int = 8
    conv_load: float = 0.25          # convert+IQ cost as fraction of realtime
    app_load: float = 0.25           # application callback cost as fraction

    # --- closed-loop coupling (see LOOP_CLOSURE.md) ---
    # "always": airspy.c:477-482, drop the newest block and resubmit anyway.
    # "block_when_full": the counterfactual "real backpressure" variant that
    # stops reaping/resubmitting while the consumer queue is full.
    resubmit_policy: str = "always"
    # 0 = event thread always runnable; else it is only scheduled this often,
    # modelling scheduling latency rather than CPU bandwidth shortage.
    event_wake_period: float = 0.0
    cpu_capacity: float = 2.0        # cores available to the two driver threads
    exogenous_load: float = 0.0      # other host work, in cores

    # --- downstream application (SDR# stand-in) ---
    app_ring_blocks: int = 4         # blocks the application can hold
    app_drain_realtime: bool = True  # application DSP drains at exactly realtime

    # --- simulation ---
    dt: float = 5e-6


@dataclass
class Disturbance:
    """A stall or slowdown injected at one stage.

    speed is the fraction of normal progress the stage makes while the
    disturbance is active. 0.0 is a hard stall; 0.5 is half speed.
    "cpu" applies to the consumer and the event thread simultaneously, which
    is the realistic case: the same core shortage delays both.
    """
    start: float
    duration: float
    where: str                       # "app" | "event" | "cpu" | "link"
    speed: float = 0.0

    @property
    def end(self) -> float:
        return self.start + self.duration


@dataclass
class Result:
    adc_lost_bytes: float = 0.0
    device_discard_bytes: float = 0.0
    host_drop_bytes: float = 0.0
    app_drop_bytes: float = 0.0

    halts: int = 0
    halt_time: float = 0.0
    rolling_entries: int = 0
    rolling_time: float = 0.0
    torn_banks: int = 0

    max_ring_occupancy: int = 0
    max_dtd_depth: int = 0
    max_consumer_depth: int = 0
    min_urb_pending: int = 0
    max_app_ring: int = 0

    # recovery: time from disturbance end until the device ring is empty again
    # and the host consumer queue is back to its pre-disturbance depth
    elasticity_restored_at: Optional[float] = None
    queue_restored_at: Optional[float] = None

    # burst measurement at the application boundary
    peak_app_rate_ratio: float = 0.0
    peak_reap_rate_ratio: float = 0.0

    max_sample_age: float = 0.0
    backpressure_blocks: int = 0
    first_adc_loss_at: Optional[float] = None

    def rf_lost_ms(self, adc_rate: float) -> float:
        total = self.adc_lost_bytes + self.device_discard_bytes + self.host_drop_bytes
        return 1000.0 * total / adc_rate


class Sim:
    def __init__(self, p: Params, disturbances: Optional[List[Disturbance]] = None):
        self.p = p
        self.dist = disturbances or []
        self.r = Result()

        # source
        self.fifo = 0.0

        # device ring
        self.bank_state = [FREE] * p.n_banks
        self.dma_index = 0
        self.bank_fill = 0.0
        self.halted = False
        self.protected_index = 0
        self.next_submit = 0

        # rolling-mode state (policy == "rolling")
        self.in_rolling = False
        self.private_index = 0
        self.private_fill = 0.0

        # dTD queue: list of bank indices, head is being served
        self.dtd: List[int] = []
        self.dtd_head_remaining = 0.0
        self.prime_ready_at = 0.0

        # host URBs. Each entry is [bytes filled, capture time of first byte]
        # so sample age can be followed all the way to the application.
        self.urb_inflight: List[List[float]] = [[0.0, -1.0] for _ in range(p.n_urbs)]
        self.urb_done: List[float] = []                     # capture times, awaiting reap
        self.reap_credit = 0.0

        # consumer queue holds capture times; the slot of the block being
        # serviced is released only after the callback returns.
        self.consumer_q: List[float] = []
        self.consumer_serving = False
        self.serving_ts = 0.0
        self.conv_remaining = 0.0
        self.app_remaining = 0.0
        self.ages: List[Tuple[float, float]] = []           # (delivery time, age)
        self.next_wake = 0.0
        # event timestamps, used to test for self-sustaining oscillation
        # contiguous dropout runs: a single consecutive gap is far kinder to
        # downstream DSP than the same total spread over many micro-gaps
        self.loss_runs: List[Tuple[float, float]] = []
        self._in_run = False
        self._run_start = 0.0
        self._last_loss_total = 0.0
        self.trace: List[Tuple[float, int, int, int, int]] = []
        self._trace_acc = 0.0
        self.halt_times: List[float] = []
        self.drop_times: List[float] = []
        self.roll_times: List[float] = []

        # application ring
        self.app_ring = 0.0                                 # bytes held
        self.app_delivered_window = 0.0
        self.reap_window = 0.0

        self.t = 0.0
        self.block_service = (p.conv_load + p.app_load) * (p.urb_bytes / p.adc_rate)
        self.t_conv = p.conv_load * (p.urb_bytes / p.adc_rate)

        # steady-state reference captured before the first disturbance
        self._dist_end = max((d.end for d in self.dist), default=0.0)
        self._settled_ring = 0
        self._settled_consumer = 0

    # -- helpers ------------------------------------------------------------
    def _cpu_share(self, who: str) -> float:
        """Fair-share CPU available to a driver thread this instant.

        Both driver threads are single-threaded, so each can use at most one
        core. When the two are runnable at the same time and capacity is short,
        they split what is left after exogenous host work.
        """
        p = self.p
        runnable = 0
        if self.urb_done:
            runnable += 1
        if self.consumer_q or self.consumer_serving:
            runnable += 1
        if runnable == 0:
            return 1.0
        avail = max(0.0, p.cpu_capacity - p.exogenous_load)
        return min(1.0, avail / runnable)

    def _speed(self, where: str) -> float:
        s = 1.0
        for d in self.dist:
            if not (d.start <= self.t < d.end):
                continue
            if d.where == where or (d.where == "cpu" and where in ("app", "event")):
                s = min(s, d.speed)
        return s

    def _transport_slots(self) -> range:
        if self.p.policy == "rolling":
            return range(self.p.transport_banks)
        return range(self.p.n_banks)

    def _ring_occupancy(self) -> int:
        return sum(1 for i in self._transport_slots() if self.bank_state[i] != FREE)

    def _next_index(self, i: int) -> int:
        limit = self.p.transport_banks if self.p.policy == "rolling" else self.p.n_banks
        n = i + 1
        return 0 if n >= limit else n

    # -- stages -------------------------------------------------------------
    def _source_and_dma(self, dt: float) -> None:
        p = self.p
        produced = p.adc_rate * dt

        if self.in_rolling:
            # GPDMA keeps running through the private pair. Older private
            # history is overwritten; RF time is lost but nothing overflows.
            self.private_fill += produced
            while self.private_fill >= p.bank_bytes:
                self.private_fill -= p.bank_bytes
                self.private_index = 1 - self.private_index
                # the bank we just wrapped over discarded a bank of history
                self.r.device_discard_bytes += p.bank_bytes
            return

        if self.halted:
            # DMA ignores requests; the ADC keeps converting into the FIFO.
            self.fifo += produced
            if self.fifo > p.fifo_bytes:
                if self.r.first_adc_loss_at is None:
                    self.r.first_adc_loss_at = self.t
                self.r.adc_lost_bytes += self.fifo - p.fifo_bytes
                self.fifo = p.fifo_bytes
            return

        # drain FIFO first, then live production
        avail = self.fifo + produced
        self.fifo = 0.0

        while avail > 0.0:
            room = p.bank_bytes - self.bank_fill
            take = min(room, avail)
            self.bank_fill += take
            avail -= take
            if self.bank_fill < p.bank_bytes:
                break
            # bank complete
            self.bank_fill = 0.0
            idx = self.dma_index
            if p.policy == "overwrite" and self.bank_state[idx] != FREE:
                # DMA has just written over a bank whose previous contents were
                # never delivered. That RF time is gone either way; if a live
                # dTD was still sending it, the host also receives a torn block
                # with no marker.
                self.r.device_discard_bytes += p.bank_bytes
                if self.bank_state[idx] == SUBMITTED:
                    self.r.torn_banks += 1
            self.bank_state[idx] = READY
            self.dma_index = self._next_index(idx)

            if p.policy == "halt":
                # airspy_m4.c: the bank two ahead must be free now, else halt
                prot = self._next_index(self._next_index(idx))
                if self.bank_state[prot] != FREE:
                    self.halted = True
                    self.protected_index = prot
                    self.r.halts += 1
                    self.halt_times.append(self.t)
                    break
            elif p.policy == "rolling":
                if self._ring_occupancy() >= p.rolling_trigger:
                    self.in_rolling = True
                    self.r.rolling_entries += 1
                    self.roll_times.append(self.t)
                    self.private_fill = 0.0
                    break
            elif p.policy == "overwrite":
                pass

        if self.halted and avail > 0.0:
            self.fifo = min(avail, p.fifo_bytes)
            self.r.adc_lost_bytes += max(0.0, avail - p.fifo_bytes)

    def _device_resume(self) -> None:
        p = self.p
        if self.halted:
            if self.bank_state[self.protected_index] == FREE:
                self.halted = False
                # airspy_m4.c clears the ADCHS overflow STATUS on resume, not
                # the FIFO contents. Samples still held in the FIFO are valid
                # and get DMA'd out, so they must be drained rather than
                # dropped here -- zeroing them silently deleted RF time and
                # under-reported loss in the repeated-short-halt regime.
        elif self.in_rolling:
            # Reentry starts a new transport/ownership epoch. That requires the
            # whole transport ring to be retired, so DMA order and submission
            # order restart coherently at index 0.
            if self._ring_occupancy() <= p.rolling_reentry:
                self.in_rolling = False
                self.bank_fill = 0.0
                self.dma_index = 0
                self.next_submit = 0
                # the private rolling history is intentionally discarded
                self.r.device_discard_bytes += self.private_fill
                self.private_fill = 0.0

    def _m0_submit(self) -> None:
        p = self.p
        limit = p.transport_banks if p.policy == "rolling" else p.n_banks
        guard = 0
        while len(self.dtd) < p.dtd_pool and guard < limit:
            guard += 1
            idx = self.next_submit % limit
            if self.bank_state[idx] != READY:
                break
            self.bank_state[idx] = SUBMITTED
            if not self.dtd:
                self.dtd_head_remaining = float(p.bank_bytes)
                # endpoint was idle: M0 must prime it by hand
                self.prime_ready_at = self.t + p.m0_prime_latency
            self.dtd.append(idx)
            self.next_submit = (self.next_submit + 1) % limit

    def _link(self, dt: float) -> None:
        p = self.p
        if self.t < self.prime_ready_at:
            return
        budget = p.mu_usb * self._speed("link") * dt
        while budget > 0.0 and self.dtd:
            # host must have an in-flight URB with room, else the endpoint NAKs
            slot = None
            for i, urb in enumerate(self.urb_inflight):
                if urb[0] < p.urb_bytes:
                    slot = i
                    break
            if slot is None:
                break
            urb = self.urb_inflight[slot]
            if urb[1] < 0.0:
                # first byte of this host block: it was captured one device-ring
                # transit ago, which is what the age accounting must charge.
                urb[1] = self.t - (self._ring_occupancy() * p.bank_bytes) / p.adc_rate
            room = p.urb_bytes - urb[0]
            take = min(budget, room, self.dtd_head_remaining)
            urb[0] += take
            self.dtd_head_remaining -= take
            budget -= take

            if self.dtd_head_remaining <= 1e-9:
                idx = self.dtd.pop(0)
                self.bank_state[idx] = FREE
                self.dtd_head_remaining = float(p.bank_bytes) if self.dtd else 0.0

            if urb[0] >= p.urb_bytes:
                self.urb_inflight.pop(slot)
                self.urb_done.append(urb[1])

    def _event_thread(self, dt: float) -> None:
        p = self.p
        speed = self._speed("event") * self._cpu_share("event")
        if speed <= 0.0:
            return

        if p.event_wake_period > 0.0:
            # The thread is runnable but not scheduled until its next wake.
            if self.t < self.next_wake:
                return
            self.next_wake += p.event_wake_period
            self.reap_credit += p.event_wake_period * speed
        else:
            self.reap_credit += dt * speed

        while self.urb_done and self.reap_credit >= p.t_reap:
            occupancy = len(self.consumer_q) + (1 if self.consumer_serving else 0)
            if occupancy >= p.consumer_slots and p.resubmit_policy == "block_when_full":
                # Counterfactual: honour consumer backpressure by refusing to
                # reap. The URB is never resubmitted, so host-side capacity
                # falls and the device is pushed back. This is the edge the
                # real driver deliberately does not have.
                self.r.backpressure_blocks += 1
                return
            self.reap_credit -= p.t_reap
            ts = self.urb_done.pop(0)
            if occupancy < p.consumer_slots:
                self.consumer_q.append(ts)
            else:
                # airspy.c: queue full -> discard the newest complete block,
                # count it, and resubmit the transfer anyway. The queue keeps
                # its older contents, so the standing latency does not fall.
                self.r.host_drop_bytes += p.urb_bytes
                self.drop_times.append(self.t)
            self.reap_window += p.urb_bytes
            # immediate resubmission: the URB returns to the in-flight pool
            self.urb_inflight.append([0.0, -1.0])
        if not self.urb_done:
            self.reap_credit = min(self.reap_credit, p.t_reap)

    def _consumer(self, dt: float) -> None:
        """One consumer thread: dequeue -> convert/IQ -> application callback.

        The queue slot is released only after the callback returns, matching
        `received_buffer_count--` after `device->callback(...)` in airspy.c.
        Conversion always makes progress; only the application callback is
        subject to an "app" stall.
        """
        p = self.p
        budget = dt
        while budget > 0.0:
            if not self.consumer_serving:
                if not self.consumer_q:
                    return
                self.consumer_serving = True
                self.serving_ts = self.consumer_q.pop(0)
                self.conv_remaining = self.t_conv
                self.app_remaining = self.block_service - self.t_conv

            if self.conv_remaining > 0.0:
                step = min(budget, self.conv_remaining)
                self.conv_remaining -= step
                budget -= step
                continue

            speed = self._speed("app") * self._cpu_share("consumer")
            if speed <= 0.0:
                return                      # callback stuck; slot stays held
            step = min(budget, self.app_remaining / speed)
            self.app_remaining -= step * speed
            budget -= step
            if self.app_remaining <= 1e-12:
                self.consumer_serving = False
                age = self.t - self.serving_ts
                self.ages.append((self.t, age))
                self.r.max_sample_age = max(self.r.max_sample_age, age)
                self._deliver_to_app(p.urb_bytes)

    def _deliver_to_app(self, nbytes: float) -> None:
        p = self.p
        cap = p.app_ring_blocks * p.urb_bytes
        if self.app_ring + nbytes > cap:
            spill = self.app_ring + nbytes - cap
            self.r.app_drop_bytes += spill
            self.app_ring = cap
        else:
            self.app_ring += nbytes
        self.app_delivered_window += nbytes

    def _app_drain(self, dt: float) -> None:
        if self.p.app_drain_realtime:
            self.app_ring = max(0.0, self.app_ring - self.p.adc_rate * dt)

    # -- driver -------------------------------------------------------------
    def run(self, duration: float) -> Result:
        p = self.p
        dt = p.dt
        steps = int(duration / dt)
        window = 0.0
        # Long enough to average over several 256 KiB delivery quanta, so the
        # metric reports a sustained catch-up rate and not the block quantum.
        window_len = 25e-3

        for _ in range(steps):
            self._device_resume()
            self._source_and_dma(dt)
            self._m0_submit()
            self._link(dt)
            self._event_thread(dt)
            self._consumer(dt)
            self._app_drain(dt)

            if self.halted:
                self.r.halt_time += dt
            if self.in_rolling:
                self.r.rolling_time += dt

            lost_now = self.r.adc_lost_bytes + self.r.device_discard_bytes
            losing = lost_now > self._last_loss_total + 1e-9
            self._last_loss_total = lost_now
            if losing and not self._in_run:
                self._in_run = True
                self._run_start = self.t
            elif not losing and self._in_run:
                self._in_run = False
                self.loss_runs.append((self._run_start, self.t))

            occ = self._ring_occupancy()
            cq = len(self.consumer_q) + (1 if self.consumer_serving else 0)
            # stage-4 backlog is len(urb_done): completed host transfers
            # waiting to be reaped. That is the ripple we are watching.
            self._trace_acc += dt
            if self._trace_acc >= 5e-4:
                self._trace_acc = 0.0
                self.trace.append((self.t, len(self.urb_done), occ, cq,
                                   1 if (self.halted or self.in_rolling) else 0))
            self.r.max_ring_occupancy = max(self.r.max_ring_occupancy, occ)
            self.r.max_dtd_depth = max(self.r.max_dtd_depth, len(self.dtd))
            self.r.max_consumer_depth = max(self.r.max_consumer_depth, cq)
            self.r.max_app_ring = max(self.r.max_app_ring,
                                      int(self.app_ring / p.urb_bytes))

            # burst measurement at the application boundary
            window += dt
            if window >= window_len:
                ratio = (self.app_delivered_window / window) / p.adc_rate
                self.r.peak_app_rate_ratio = max(self.r.peak_app_rate_ratio, ratio)
                rr = (self.reap_window / window) / p.adc_rate
                self.r.peak_reap_rate_ratio = max(self.r.peak_reap_rate_ratio, rr)
                self.app_delivered_window = 0.0
                self.reap_window = 0.0
                window = 0.0

            # settle references, taken just before the first disturbance
            if self.dist and self.t < self.dist[0].start:
                self._settled_ring = occ
                self._settled_consumer = cq
                self.r.min_urb_pending = len(self.urb_inflight)
            elif self.dist:
                self.r.min_urb_pending = min(self.r.min_urb_pending,
                                             len(self.urb_inflight))

            if self.dist and self.t > self._dist_end:
                if (self.r.elasticity_restored_at is None
                        and occ <= self._settled_ring
                        and not self.halted and not self.in_rolling
                        and len(self.dtd) <= 1):
                    self.r.elasticity_restored_at = self.t
                if (self.r.queue_restored_at is None
                        and cq <= self._settled_consumer + 1):
                    self.r.queue_restored_at = self.t

            self.t += dt

        return self.r


# ---------------------------------------------------------------------------
# Analytic companions
# ---------------------------------------------------------------------------

def drain_stretch(adc_rate: float, mu: float) -> float:
    """Time to drain a queue built during a stall of length T, in units of T.

    rho = lambda/mu ; excess drain rate is (mu - lambda) ; volume is lambda*T
    => drain time = T * rho/(1-rho)
    """
    if mu <= adc_rate:
        return math.inf
    rho = adc_rate / mu
    return rho / (1.0 - rho)


def stage_elasticity_ms(p: Params) -> List[Tuple[str, float, float]]:
    """(stage, bytes, milliseconds of RF time) for each buffering stage."""
    out = [
        ("ADCHS FIFO", p.fifo_bytes, 1000 * p.fifo_bytes / p.adc_rate),
        ("SRAM bank ring", p.n_banks * p.bank_bytes,
         1000 * p.n_banks * p.bank_bytes / p.adc_rate),
        ("dTD queue (aliases ring)", p.dtd_pool * p.bank_bytes,
         1000 * p.dtd_pool * p.bank_bytes / p.adc_rate),
        ("host URB pool", p.n_urbs * p.urb_bytes,
         1000 * p.n_urbs * p.urb_bytes / p.adc_rate),
        ("consumer queue", p.consumer_slots * p.urb_bytes,
         1000 * p.consumer_slots * p.urb_bytes / p.adc_rate),
        ("application ring", p.app_ring_blocks * p.urb_bytes,
         1000 * p.app_ring_blocks * p.urb_bytes / p.adc_rate),
    ]
    return out


# ---------------------------------------------------------------------------
# Experiments
# ---------------------------------------------------------------------------

def hdr(title: str) -> None:
    print()
    print("=" * 78)
    print(title)
    print("=" * 78)


def e0_geometry(p: Params) -> None:
    hdr("E0  stage geometry and elasticity")
    print(f"{'stage':<28}{'bytes':>12}{'ms of RF':>12}")
    for name, b, ms in stage_elasticity_ms(p):
        print(f"{name:<28}{b:>12,.0f}{ms:>12.3f}")
    print()
    print(f"bank time            = {1000*p.bank_bytes/p.adc_rate:.4f} ms")
    print(f"URB time             = {1000*p.urb_bytes/p.adc_rate:.4f} ms")
    print(f"URB : bank ratio     = {p.urb_bytes//p.bank_bytes} banks per host completion")
    print()
    print("drain stretch rho/(1-rho) as a function of effective link rate:")
    print(f"{'mu (MB/s)':>12}{'rho':>10}{'stretch':>10}")
    for mu in (40.5, 41, 42, 43, 45, 48, 50, 53.2):
        s = drain_stretch(p.adc_rate, mu * 1e6)
        print(f"{mu:>12.1f}{p.adc_rate/(mu*1e6):>10.3f}{s:>10.2f}")
    print()
    print("same table with 12-bit packing enabled (lambda = 30 MB/s):")
    print(f"{'mu (MB/s)':>12}{'rho':>10}{'stretch':>10}")
    for mu in (40.5, 43, 45, 48, 53.2):
        s = drain_stretch(ADC_RATE_PACKED, mu * 1e6)
        print(f"{mu:>12.1f}{ADC_RATE_PACKED/(mu*1e6):>10.3f}{s:>10.2f}")


def e1_steady(p: Params, quick: bool) -> None:
    hdr("E1  steady state vs effective link rate (no disturbance)")
    print(f"{'mu (MB/s)':>10}{'ring max':>10}{'dTD max':>9}{'cq max':>8}"
          f"{'halts':>7}{'adc lost ms':>13}{'host drop ms':>14}")
    for mu in (41, 42, 43, 45, 48, 53.2):
        q = Params(**{**p.__dict__, "mu_usb": mu * 1e6})
        s = Sim(q)
        r = s.run(0.3 if quick else 0.6)
        print(f"{mu:>10.1f}{r.max_ring_occupancy:>10}{r.max_dtd_depth:>9}"
              f"{r.max_consumer_depth:>8}{r.halts:>7}"
              f"{1000*r.adc_lost_bytes/q.adc_rate:>13.3f}"
              f"{1000*r.host_drop_bytes/q.adc_rate:>14.3f}")


def e2_app_stall(p: Params, quick: bool) -> None:
    hdr("E2  application-callback stall  (the intended firewall)")
    print("A stall in the SDR#-side callback. The driver is designed to convert")
    print("this into counted host-block drops and no device-side effect.")
    print()
    print(f"{'stall ms':>10}{'adc ms':>9}{'devdisc ms':>12}{'hostdrop ms':>13}"
          f"{'appdrop ms':>12}{'halts':>7}{'ring max':>10}{'recov ms':>10}")
    stalls = (5, 20, 52, 60, 100, 200) if quick else (1, 5, 20, 40, 52, 60, 100, 200, 400)
    for ms in stalls:
        q = Params(**p.__dict__)
        d = [Disturbance(start=0.20, duration=ms / 1000.0, where="app")]
        s = Sim(q, d)
        r = s.run(0.20 + ms / 1000.0 + 0.60)
        recov = ((r.queue_restored_at - d[0].end) * 1000.0
                 if r.queue_restored_at else float("nan"))
        print(f"{ms:>10}{1000*r.adc_lost_bytes/q.adc_rate:>9.3f}"
              f"{1000*r.device_discard_bytes/q.adc_rate:>12.3f}"
              f"{1000*r.host_drop_bytes/q.adc_rate:>13.3f}"
              f"{1000*r.app_drop_bytes/q.adc_rate:>12.3f}"
              f"{r.halts:>7}{r.max_ring_occupancy:>10}{recov:>10.2f}")


def e3_event_stall(p: Params, quick: bool) -> None:
    hdr("E3  libusb event-thread stall  (the leak path)")
    print("The completion thread stops reaping and resubmitting. Host URB depth")
    print("collapses, the endpoint runs out of host-side room, and the device")
    print("ring is the only remaining elasticity.")
    print()
    for policy in ("halt", "rolling"):
        print(f"--- device policy: {policy} ---")
        print(f"{'stall ms':>10}{'adc ms':>9}{'devdisc ms':>12}{'hostdrop ms':>13}"
              f"{'halts':>7}{'halt ms':>9}{'RF lost ms':>12}{'gain':>8}{'elast ms':>10}")
        stalls = ((20, 60, 100, 110, 130) if quick else
                  (5, 20, 60, 90, 100, 105, 110, 115, 120, 130, 150, 200))
        for ms in stalls:
            q = Params(**{**p.__dict__, "policy": policy})
            d = [Disturbance(start=0.20, duration=ms / 1000.0, where="event")]
            s = Sim(q, d)
            r = s.run(0.20 + ms / 1000.0 + 0.60)
            rf = r.rf_lost_ms(q.adc_rate)
            elast = ((r.elasticity_restored_at - d[0].end) * 1000.0
                     if r.elasticity_restored_at else float("nan"))
            print(f"{ms:>10}{1000*r.adc_lost_bytes/q.adc_rate:>9.3f}"
                  f"{1000*r.device_discard_bytes/q.adc_rate:>12.3f}"
                  f"{1000*r.host_drop_bytes/q.adc_rate:>13.3f}"
                  f"{r.halts:>7}{1000*r.halt_time:>9.3f}"
                  f"{rf:>12.3f}{rf/ms:>8.2f}{elast:>10.2f}")
        print()


def mean_age_ms(sim: "Sim", lo: float, hi: float) -> float:
    vals = [a for (t, a) in sim.ages if lo <= t < hi]
    return 1000.0 * sum(vals) / len(vals) if vals else float("nan")


def e4_repetition(p: Params, quick: bool) -> None:
    hdr("E4  repeated disturbance  (the multiplication test)")
    print("If congestion did not multiply, loss would be linear in the number")
    print("of disturbances and independent of their spacing.")
    print()
    print("--- 4a: device-reaching stalls (120 ms event-thread outage x 5) ---")
    print("A 120 ms outage exhausts the 105 ms host URB pool and reaches the")
    print("device ring. The question is whether the NEXT one costs more.")
    print()
    for policy in ("halt", "rolling"):
        q0 = Params(**{**p.__dict__, "policy": policy})
        d0 = [Disturbance(start=0.20, duration=0.120, where="event")]
        ref = Sim(q0, d0).run(0.9).rf_lost_ms(q0.adc_rate)
        print(f"  policy={policy}  isolated single-stall RF loss = {ref:.3f} ms")
        print(f"{'  interval ms':>14}{'RF lost ms':>12}{'per stall':>11}"
              f"{'vs isolated':>13}{'halts':>7}{'adc ms':>9}")
        intervals = (130, 200, 400) if quick else (125, 130, 140, 160, 200, 300, 400)
        for I in intervals:
            q = Params(**{**p.__dict__, "policy": policy})
            n = 5
            d = [Disturbance(start=0.20 + k * I / 1000.0, duration=0.120,
                             where="event") for k in range(n)]
            r = Sim(q, d).run(0.20 + n * I / 1000.0 + 0.4)
            rf = r.rf_lost_ms(q.adc_rate)
            mult = (rf / n) / ref if ref > 1e-9 else float("inf")
            print(f"{I:>14}{rf:>12.3f}{rf/n:>11.3f}{mult:>13.2f}{r.halts:>7}"
                  f"{1000*r.adc_lost_bytes/q.adc_rate:>9.3f}")
        print()

    print("--- 4b: consumer-side compounding at high consumer utilisation ---")
    print("30 ms application stalls repeated at interval I, with the consumer")
    print("thread already loaded to rho_c of realtime. rho_c is convert+IQ+")
    print("callback cost divided by the realtime budget of one 256 KiB block.")
    print()
    for rho_c in (0.5, 0.8, 0.9):
        load = rho_c / 2.0
        q0 = Params(**{**p.__dict__, "conv_load": load, "app_load": load})
        d0 = [Disturbance(start=0.30, duration=0.030, where="app")]
        s0 = Sim(q0, d0)
        ref = s0.run(1.2).rf_lost_ms(q0.adc_rate)
        shadow = 30.0 * rho_c / (1.0 - rho_c)
        print(f"  rho_c={rho_c:.1f}  isolated single-stall RF loss = {ref:.3f} ms"
              f"   predicted recovery shadow = {shadow:.0f} ms")
        print(f"{'  interval ms':>14}{'RF lost ms':>12}{'per stall':>11}"
              f"{'interaction ms':>16}{'inside shadow':>15}")
        intervals = (60, 120, 300) if quick else (40, 50, 60, 80, 120, 200, 300)
        for I in intervals:
            q = Params(**{**p.__dict__, "conv_load": load, "app_load": load})
            n = 5
            d = [Disturbance(start=0.30 + k * I / 1000.0, duration=0.030,
                             where="app") for k in range(n)]
            s = Sim(q, d)
            r = s.run(0.30 + n * I / 1000.0 + 0.5)
            rf = r.rf_lost_ms(q.adc_rate)
            interaction = rf - n * ref
            inside = "yes" if I < shadow + 30.0 else "no"
            print(f"{I:>14}{rf:>12.3f}{rf/n:>11.3f}{interaction:>16.3f}"
                  f"{inside:>15}")
        print()


def e9_consumer_load(p: Params, quick: bool) -> None:
    hdr("E9  consumer utilisation, drain stretch, and the latency ratchet")
    print("One 40 ms application stall. rho_c is the consumer thread's steady")
    print("utilisation. 'age' is the delivered sample age at the application")
    print("callback. 'latency recovery' is how long age stayed above 1.2x its")
    print("baseline after the stall ended; 'predicted' is T*rho_c/(1-rho_c).")
    print()
    print(f"{'rho_c':>7}{'RF lost ms':>12}{'age base ms':>13}{'age peak ms':>13}"
          f"{'latency recovery ms':>21}{'predicted':>11}")
    loads = (0.3, 0.5, 0.7, 0.8, 0.9, 0.95)
    stall_ms = 40.0
    for rho_c in loads:
        load = rho_c / 2.0
        q = Params(**{**p.__dict__, "conv_load": load, "app_load": load})
        d = [Disturbance(start=0.30, duration=stall_ms / 1000.0, where="app")]
        s = Sim(q, d)
        r = s.run(2.4)
        base = mean_age_ms(s, 0.20, 0.30)
        peak = 1000.0 * r.max_sample_age
        # latency recovery: last delivery whose age still exceeded 1.2x baseline
        end = d[0].end
        late = [t for (t, a) in s.ages if t > end and 1000.0 * a > 1.2 * base]
        rec = (max(late) - end) * 1000.0 if late else 0.0
        predicted = stall_ms * rho_c / (1.0 - rho_c)
        print(f"{rho_c:>7.2f}{r.rf_lost_ms(q.adc_rate):>12.3f}{base:>13.2f}"
              f"{peak:>13.2f}{rec:>21.1f}{predicted:>11.1f}")


def e5_burst(p: Params, quick: bool) -> None:
    hdr("E5  burst propagation to the downstream application")
    print("Any stall becomes a gap followed by a catch-up burst at line rate.")
    print("Measured: peak 1 ms delivery rate at the application boundary, as a")
    print("multiple of realtime, and whether that burst overruns a downstream")
    print("ring sized for the mean.")
    print()
    print(f"{'stall ms':>10}{'where':>8}{'peak/realtime':>15}"
          f"{'app ring max':>14}{'app drop ms':>13}")
    cases = [(5, "event"), (20, "event"), (20, "app"), (60, "app")]
    if not quick:
        cases += [(100, "app"), (200, "app"), (60, "event")]
    for ms, where in cases:
        q = Params(**p.__dict__)
        d = [Disturbance(start=0.20, duration=ms / 1000.0, where=where)]
        s = Sim(q, d)
        r = s.run(0.20 + ms / 1000.0 + 0.6)
        print(f"{ms:>10}{where:>8}{r.peak_app_rate_ratio:>15.3f}"
              f"{r.max_app_ring:>14}{1000*r.app_drop_bytes/q.adc_rate:>13.3f}")


def e6_packing(p: Params, quick: bool) -> None:
    hdr("E6  headroom sensitivity: unpacked 40 MB/s vs packed 30 MB/s")
    print("Same device policy, only the payload rate differs. This isolates the")
    print("effect of rho on recovery.")
    print()
    print("Each case is stalled 25 ms beyond its own host URB pool depth, so")
    print("the device is reached by the same margin and only rho differs.")
    print()
    print(f"{'lambda MB/s':>12}{'mu MB/s':>9}{'rho':>7}{'pool ms':>9}"
          f"{'stall ms':>10}{'adc ms':>9}{'halts':>7}{'elast ms':>10}"
          f"{'analytic stretch':>18}")
    for lam in (ADC_RATE_UNPACKED, ADC_RATE_PACKED):
        for mu in (43e6, 45e6, 48e6):
            q = Params(**{**p.__dict__, "adc_rate": lam, "mu_usb": mu})
            pool_ms = 1000.0 * q.n_urbs * q.urb_bytes / lam
            stall = (pool_ms + 25.0) / 1000.0
            d = [Disturbance(start=0.05, duration=stall, where="event")]
            s = Sim(q, d)
            r = s.run(0.05 + stall + 0.5)
            elast = ((r.elasticity_restored_at - d[0].end) * 1000.0
                     if r.elasticity_restored_at else float("nan"))
            print(f"{lam/1e6:>12.0f}{mu/1e6:>9.0f}{lam/mu:>7.3f}{pool_ms:>9.1f}"
                  f"{1000*stall:>10.1f}{1000*r.adc_lost_bytes/lam:>9.3f}"
                  f"{r.halts:>7}{elast:>10.2f}{drain_stretch(lam, mu):>18.2f}")


def e7_policies(p: Params, quick: bool) -> None:
    hdr("E7  device policy comparison under a 130 ms event-thread stall")
    print("130 ms exceeds the 104.9 ms host URB pool, so the device ring is")
    print("genuinely reached and the policy actually matters.")
    print()
    print(f"{'policy':>12}{'adc ms':>9}{'devdisc ms':>12}{'hostdrop ms':>13}"
          f"{'RF lost ms':>12}{'torn':>6}{'halt ms':>9}{'elast ms':>10}")
    for policy in ("halt", "rolling", "overwrite"):
        q = Params(**{**p.__dict__, "policy": policy})
        if policy == "overwrite":
            q.n_banks = 2
            q.dtd_pool = 1
        d = [Disturbance(start=0.10, duration=0.130, where="event")]
        s = Sim(q, d)
        r = s.run(0.9)
        elast = ((r.elasticity_restored_at - d[0].end) * 1000.0
                 if r.elasticity_restored_at else float("nan"))
        print(f"{policy:>12}{1000*r.adc_lost_bytes/q.adc_rate:>9.3f}"
              f"{1000*r.device_discard_bytes/q.adc_rate:>12.3f}"
              f"{1000*r.host_drop_bytes/q.adc_rate:>13.3f}"
              f"{r.rf_lost_ms(q.adc_rate):>12.3f}{r.torn_banks:>6}"
              f"{1000*r.halt_time:>9.3f}{elast:>10.2f}")
    print()
    print("Note: the two-bank 'overwrite' row is the vanilla-style geometry")
    print("(2 banks, 1 dTD) and is included for contrast only.")


def e8_ring_depth(p: Params, quick: bool) -> None:
    hdr("E8  marginal value of device ring depth against host-side stalls")
    print("Longest event-thread outage that still causes zero ADC/FIFO loss,")
    print("as a function of bank count. The host URB pool already covers")
    print(f"{1000*p.n_urbs*p.urb_bytes/p.adc_rate:.2f} ms, so this measures what "
          "each extra bank adds on top.")
    print()
    print(f"{'banks':>7}{'ring ms':>9}{'longest lossless stall ms':>27}"
          f"{'marginal ms/bank':>18}")
    prev = None
    # n_banks=2 is degenerate under the halt policy: the protected bank
    # index (i+2) wraps onto the bank just completed, so it halts immediately.
    counts = (4, 8, 10, 16) if quick else (4, 6, 8, 10, 12, 16, 20)
    base = 1000.0 * p.n_urbs * p.urb_bytes / p.adc_rate
    for n in counts:
        survived = 0.0
        ms = base - 4.0
        while ms < base + 12.0:
            q = Params(**{**p.__dict__, "n_banks": n, "dtd_pool": n})
            d = [Disturbance(start=0.05, duration=ms / 1000.0, where="event")]
            r = Sim(q, d).run(0.05 + ms / 1000.0 + 0.05)
            if r.adc_lost_bytes > 1.0:
                break
            survived = ms
            ms += 0.25
        marg = ((survived - prev[1]) / (n - prev[0])) if prev else float("nan")
        prev = (n, survived)
        print(f"{n:>7}{1000*n*p.bank_bytes/p.adc_rate:>9.3f}{survived:>27.2f}"
              f"{marg:>18.3f}")
    print()
    print(f"one bank = {1000*p.bank_bytes/p.adc_rate:.4f} ms of RF time")


def _events_after(sim: "Sim", t0: float) -> Tuple[int, int]:
    """(device events, host drop events) occurring after t0."""
    dev = len([t for t in sim.halt_times if t > t0]) + \
          len([t for t in sim.roll_times if t > t0])
    drops = len([t for t in sim.drop_times if t > t0])
    return dev, drops


def e10_loop_closure(p: Params, quick: bool) -> None:
    hdr("E10  closed-loop test: does one trigger manufacture more events?")
    print("A single 130 ms event-thread stall at t=0.10 s, then NOTHING further")
    print("is injected. The simulation runs to 3.0 s. Any device or host event")
    print("after t=0.45 s (well past the direct recovery) is self-manufactured.")
    print()
    print(f"{'resubmit policy':>18}{'rho_c':>7}{'device ev':>11}{'drop ev':>9}"
          f"{'self-made dev':>15}{'self-made drop':>16}{'verdict':>12}")
    for policy in ("always", "block_when_full"):
        for rho_c in (0.5, 0.8, 0.95):
            load = rho_c / 2.0
            q = Params(**{**p.__dict__, "resubmit_policy": policy,
                          "conv_load": load, "app_load": load})
            d = [Disturbance(start=0.10, duration=0.130, where="event")]
            s = Sim(q, d)
            s.run(3.0)
            dev_all = len(s.halt_times) + len(s.roll_times)
            drop_all = len(s.drop_times)
            dev, drops = _events_after(s, 0.45)
            verdict = "TERMINATES" if (dev + drops) == 0 else "SUSTAINS"
            print(f"{policy:>18}{rho_c:>7.2f}{dev_all:>11}{drop_all:>9}"
                  f"{dev:>15}{drops:>16}{verdict:>12}")
    print()

    print("--- the question as asked: can the CLIENT provoke the DEVICE? ---")
    print("Trigger is purely client-side: a 200 ms application-callback stall.")
    print("rho_c > 1 means the client is persistently slower than realtime, so")
    print("its queue never drains on its own.")
    print()
    print(f"{'resubmit policy':>18}{'rho_c':>7}{'adc lost ms':>13}{'halts':>7}"
          f"{'host drop ms':>14}{'device reached':>16}")
    for policy in ("always", "block_when_full"):
        for rho_c in (0.5, 0.95, 1.05, 1.30):
            load = rho_c / 2.0
            q = Params(**{**p.__dict__, "resubmit_policy": policy,
                          "conv_load": load, "app_load": load})
            d = [Disturbance(start=0.10, duration=0.200, where="app")]
            s = Sim(q, d)
            r = s.run(3.0)
            reached = "YES" if r.adc_lost_bytes > 1.0 else "no"
            print(f"{policy:>18}{rho_c:>7.2f}"
                  f"{1000*r.adc_lost_bytes/q.adc_rate:>13.3f}{r.halts:>7}"
                  f"{1000*r.host_drop_bytes/q.adc_rate:>14.3f}{reached:>16}")
    print()
    print("'always' is the shipping policy (airspy.c:477-482). 'block_when_full'")
    print("is the counterfactual in which the host honours consumer backpressure")
    print("by not resubmitting, which is the change an unwary rewrite might make.")


def e11_loop_gain(p: Params, quick: bool) -> None:
    hdr("E11  the two loop edges, measured separately")
    print("Loop gain = A x B, where")
    print("  A = client -> device : ms of DEVICE disturbance (ADC loss +")
    print("      device discard) caused by X ms of purely client disturbance")
    print("  B = device -> client : ms of CLIENT disturbance (host drop + app")
    print("      drop) caused by X ms of purely transport disturbance")
    print("If either edge is identically zero, no closed loop exists and")
    print("recursive manufacture is impossible regardless of the other edge.")
    print()
    for policy in ("always", "block_when_full"):
        print(f"--- resubmit policy: {policy} ---")
        print(f"{'X ms':>7}{'A: device ms':>14}{'gain A':>9}"
              f"{'B: client ms':>14}{'gain B':>9}{'loop gain A*B':>15}")
        xs = (50, 100, 200) if quick else (25, 50, 100, 200, 400)
        for X in xs:
            qa = Params(**{**p.__dict__, "resubmit_policy": policy})
            da = [Disturbance(start=0.10, duration=X / 1000.0, where="app")]
            ra = Sim(qa, da).run(0.10 + X / 1000.0 + 1.0)
            a_ms = 1000.0 * (ra.adc_lost_bytes + ra.device_discard_bytes) / qa.adc_rate

            qb = Params(**{**p.__dict__, "resubmit_policy": policy})
            db = [Disturbance(start=0.10, duration=X / 1000.0, where="event")]
            rb = Sim(qb, db).run(0.10 + X / 1000.0 + 1.0)
            b_ms = 1000.0 * (rb.host_drop_bytes + rb.app_drop_bytes) / qb.adc_rate

            ga, gb = a_ms / X, b_ms / X
            print(f"{X:>7}{a_ms:>14.3f}{ga:>9.3f}{b_ms:>14.3f}{gb:>9.3f}"
                  f"{ga*gb:>15.4f}")
        print()


def e12_sched_latency(p: Params, quick: bool) -> None:
    hdr("E12  what it actually takes to close the client -> device edge")
    print("The event thread needs ~0.8% of one core. It cannot be starved of")
    print("CPU bandwidth. It can only be starved of SCHEDULING. Here it is")
    print("runnable throughout but only dispatched every 'wake period'.")
    print()
    print(f"{'wake period ms':>16}{'adc lost ms':>13}{'halts':>7}"
          f"{'host drop ms':>14}{'reaches device':>16}")
    periods = (10, 50, 90, 100, 105, 110, 130, 200) if not quick else (50, 105, 200)
    for ms in periods:
        q = Params(**{**p.__dict__, "event_wake_period": ms / 1000.0})
        s = Sim(q)
        r = s.run(1.5)
        reach = "yes" if r.adc_lost_bytes > 1.0 else "no"
        print(f"{ms:>16}{1000*r.adc_lost_bytes/q.adc_rate:>13.3f}{r.halts:>7}"
              f"{1000*r.host_drop_bytes/q.adc_rate:>14.3f}{reach:>16}")
    print()
    print("No injected disturbance at all: the wake period alone is the fault.")
    print()
    print("CPU bandwidth contention, for contrast (event thread always runnable):")
    print(f"{'cpu cores':>11}{'exogenous':>11}{'adc lost ms':>13}{'halts':>7}"
          f"{'host drop ms':>14}")
    for cap, exo in ((2.0, 0.0), (1.0, 0.0), (1.0, 0.5), (1.0, 0.9), (1.0, 0.99)):
        q = Params(**{**p.__dict__, "cpu_capacity": cap, "exogenous_load": exo})
        r = Sim(q).run(1.5)
        print(f"{cap:>11.2f}{exo:>11.2f}{1000*r.adc_lost_bytes/q.adc_rate:>13.3f}"
              f"{r.halts:>7}{1000*r.host_drop_bytes/q.adc_rate:>14.3f}")


STOCK = dict(n_banks=2, dtd_pool=1, policy="overwrite")
V5C = dict(n_banks=10, dtd_pool=10, policy="halt")


def e13_stock_vs_v5c(p: Params, quick: bool) -> None:
    hdr("E13  stock (2 banks, 1 dTD) vs V5c (10 banks, 10 dTD): prime latency")
    print("With a dTD pool of 1 the bulk-IN endpoint has no linked successor,")
    print("so the M0 must prime it by hand after EVERY bank. That turnaround is")
    print("dead air on the wire. With a linked queue the controller advances in")
    print("hardware and pays nothing.")
    print()
    bank_us = 1e6 * p.bank_bytes / p.adc_rate
    wire_us = 1e6 * p.bank_bytes / p.mu_usb
    print(f"bank RF time {bank_us:.1f} us, wire time {wire_us:.1f} us at "
          f"{p.mu_usb/1e6:.0f} MB/s => margin {bank_us-wire_us:.1f} us per bank")
    print()
    print("No injected disturbance of any kind. The only variable is the M0's")
    print("own prime latency.")
    print()
    print(f"{'prime us':>10}{'geometry':>10}{'adc lost ms':>13}{'first loss at ms':>18}"
          f"{'host drop ms':>14}{'torn':>6}")
    lats = (0, 5, 10, 20, 28, 30, 40, 60) if not quick else (0, 20, 30, 60)
    for us in lats:
        for name, geom in (("stock", STOCK), ("V5c", V5C)):
            q = Params(**{**p.__dict__, **geom, "m0_prime_latency": us / 1e6})
            r = Sim(q).run(1.0)
            first = (1000.0 * r.first_adc_loss_at
                     if r.first_adc_loss_at is not None else float("nan"))
            print(f"{us:>10}{name:>10}{1000*r.adc_lost_bytes/q.adc_rate:>13.3f}"
                  f"{first:>18.2f}{1000*r.host_drop_bytes/q.adc_rate:>14.3f}"
                  f"{r.torn_banks:>6}")


def e14_link_deficit(p: Params, quick: bool) -> None:
    hdr("E14  sustained link-rate deficit: mu below lambda")
    print("Host contention in its ordinary field form: another device, a hub,")
    print("or periodic (isochronous) reservations on the same controller pull")
    print("effective bulk rate below the 40 MB/s the ADC produces. No stall is")
    print("injected; the rate is simply short.")
    print()
    print(f"{'mu MB/s':>9}{'policy':>10}{'first ADC loss at ms':>22}"
          f"{'adc lost ms':>13}{'devdisc ms':>13}{'host drop ms':>14}{'note':>12}")
    for mu in (44, 42, 40.5, 40, 39, 37, 34, 30):
        for policy in ("halt", "rolling"):
            q = Params(**{**p.__dict__, "policy": policy, "mu_usb": mu * 1e6})
            r = Sim(q).run(2.0)
            first = (1000.0 * r.first_adc_loss_at
                     if r.first_adc_loss_at is not None else float("nan"))
            note = "deficit" if mu * 1e6 < q.adc_rate else ""
            print(f"{mu:>9.1f}{policy:>10}{first:>22.2f}"
                  f"{1000*r.adc_lost_bytes/q.adc_rate:>13.3f}"
                  f"{1000*r.device_discard_bytes/q.adc_rate:>13.3f}"
                  f"{1000*r.host_drop_bytes/q.adc_rate:>14.3f}{note:>12}")
    print()
    print("Signature to note: ADC loss with ZERO host drops means a link/device")
    print("rate shortage. Host drops with ZERO ADC loss means a slow consumer.")
    print("The two failures are distinguishable from counters alone.")


def e15_intermittent_link(p: Params, quick: bool) -> None:
    hdr("E15  intermittent bus contention and accumulation across episodes")
    print("mu drops to 30 MB/s for D ms out of every 200 ms, so the mean rate")
    print("is above lambda but individual episodes are in deficit. Buffers must")
    print("absorb each episode and drain before the next.")
    print()
    print("'deficit per episode' is the RF time the ring must absorb each time;")
    print(f"the ring only holds {1000.0*p.n_banks*p.bank_bytes/p.adc_rate:.2f} ms "
          "nominal and trips at ~2.75 ms.")
    print()
    print(f"{'D ms':>6}{'duty':>7}{'mean mu MB/s':>14}{'deficit ms':>13}{'policy':>9}"
          f"{'adc lost ms':>13}{'first loss at ms':>18}")
    for D in (5, 10, 20, 40, 60):
        duty = D / 200.0
        mean = 30.0 * duty + 43.0 * (1 - duty)
        deficit_ms = D * (40.0 - 30.0) / 40.0
        for policy in ("halt", "rolling"):
            q = Params(**{**p.__dict__, "policy": policy})
            d = [Disturbance(start=0.1 + k * 0.200, duration=D / 1000.0,
                             where="link", speed=30.0 / 43.0) for k in range(9)]
            r = Sim(q, d).run(2.0)
            first = (1000.0 * r.first_adc_loss_at
                     if r.first_adc_loss_at is not None else float("nan"))
            print(f"{D:>6}{duty:>7.2f}{mean:>14.2f}{deficit_ms:>13.2f}{policy:>9}"
                  f"{1000*r.adc_lost_bytes/q.adc_rate:>13.3f}{first:>18.2f}")


def e16_link_interruption(p: Params, quick: bool) -> None:
    hdr("E16  how long can the WIRE stall before V5c halts the ADC?")
    print("A link-side interruption: the bulk endpoint gets no service for D ms")
    print("(competing device, hub transaction translator, periodic schedule")
    print("spike, error/retry burst). The host URB pool is FULLY AVAILABLE the")
    print("whole time -- it simply cannot help, because the shortage is on the")
    print("wire, not in host buffer space.")
    print()
    ring_ms = 1000.0 * p.n_banks * p.bank_bytes / p.adc_rate
    print(f"device ring = {ring_ms:.3f} ms   host URB pool = "
          f"{1000.0*p.n_urbs*p.urb_bytes/p.adc_rate:.1f} ms (irrelevant here)")
    print()
    print(f"{'stall ms':>10}{'policy':>10}{'adc lost ms':>13}{'halts':>7}"
          f"{'devdisc ms':>12}{'host drop ms':>14}")
    for ms in (1, 2, 3, 4, 4.5, 5, 6, 8, 12, 20):
        for policy in ("halt", "rolling"):
            q = Params(**{**p.__dict__, "policy": policy})
            d = [Disturbance(start=0.10, duration=ms / 1000.0,
                             where="link", speed=0.0)]
            r = Sim(q, d).run(0.10 + ms / 1000.0 + 0.5)
            print(f"{ms:>10}{policy:>10}"
                  f"{1000*r.adc_lost_bytes/q.adc_rate:>13.4f}{r.halts:>7}"
                  f"{1000*r.device_discard_bytes/q.adc_rate:>12.4f}"
                  f"{1000*r.host_drop_bytes/q.adc_rate:>14.3f}")
    print()
    print("Compare: the same experiment done as an EVENT-THREAD stall needs")
    print("more than 104.9 ms to reach the ADC. A wire stall needs ~4 ms.")
    print("These are different failures and only one of them is rare.")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--only", type=str, default=None,
                    help="comma-separated experiment ids, e.g. e3,e4")
    args = ap.parse_args()

    p = Params()
    table = {
        "e0": e0_geometry, "e1": e1_steady, "e2": e2_app_stall,
        "e3": e3_event_stall, "e4": e4_repetition, "e5": e5_burst,
        "e6": e6_packing, "e7": e7_policies, "e8": e8_ring_depth,
        "e9": e9_consumer_load, "e10": e10_loop_closure,
        "e11": e11_loop_gain, "e12": e12_sched_latency,
        "e13": e13_stock_vs_v5c, "e14": e14_link_deficit,
        "e15": e15_intermittent_link, "e16": e16_link_interruption,
    }
    ids = args.only.split(",") if args.only else list(table)
    for i in ids:
        fn = table[i.strip()]
        if i.strip() == "e0":
            fn(p)
        else:
            fn(p, args.quick)


if __name__ == "__main__":
    main()
