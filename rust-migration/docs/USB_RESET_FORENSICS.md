# USB Reset / Restart Forensics

Status: first cancellation/order fix applied; instrumentation still pending.
Date: 2026-07-01.

## Problem Statement

The Rust driver is functionally working and has passed real application testing,
but stream stop/restart feels slower than the C++ driver and can produce an
audible pop. The goal of this project is to understand the timing and USB
semantics well enough to make a small, evidence-backed change rather than tuning
by feel.

## Current Rust Sequence

`rust-migration/libairspyhf/src/device.rs`:

1. `start_streaming`
   - set `streaming = true`
   - reset the IQ balancer optimal point
   - vendor request `AIRSPYHF_RECEIVER_MODE = OFF`
   - claim interface 0
   - `clear_halt_ep(0x80 | AIRSPYHF_ENDPOINT_IN)`
   - vendor request `AIRSPYHF_RECEIVER_MODE = ON`
   - spawn consumer thread
   - spawn producer thread
2. `producer_proc`
   - `iface.set_alt_setting(1)`
   - open bulk IN endpoint `0x80 | AIRSPYHF_ENDPOINT_IN`
   - `ep.clear_halt()`
   - submit `RAW_BUFFER_COUNT` bulk IN transfers
   - loop on `ep.wait_next_complete(Duration::from_millis(10))`
   - on stop/generation mismatch, call `Endpoint::cancel_all()`, drain returned
     completions briefly, then exit
   - on endpoint stall, cancel/drain before calling `ep.clear_halt()`
3. `stop_streaming`
   - set `stop_requested = true`
   - vendor request `AIRSPYHF_RECEIVER_MODE = OFF`
   - increment `generation`
   - join producer
   - join consumer unless called from callback
   - clear stream flags

Important detail: the producer used to sit inside a one-second
`wait_next_complete()` call when stop was requested. `nusb` documents that
`wait_next_complete(timeout)` does not cancel a transfer on timeout. Patch 1
reduced that polling interval and added explicit cancellation/draining on exit.

## C++ Reference Sequence

`libairspyhf/src/airspyhf.cpp`:

1. `airspyhf_start`
   - reset dropped buffer counters and rotation vector
   - `AIRSPYHF_RECEIVER_MODE = OFF`
   - `libusb_clear_halt(IN | AIRSPYHF_ENDPOINT_IN)`
   - `AIRSPYHF_RECEIVER_MODE = ON`
   - create I/O threads
2. `create_io_threads`
   - mark `streaming = true`
   - submit all libusb transfers before creating threads
   - create consumer thread
   - create transfer/event thread
3. `airspyhf_stop`
   - set `stop_requested = true`
   - `AIRSPYHF_RECEIVER_MODE = OFF`
   - `kill_io_threads`
   - on non-Windows, `libusb_interrupt_event_handler`
4. `kill_io_threads`
   - clear `stop_requested`, clear `streaming`
   - call `cancel_transfers`
   - signal the consumer condition variable
   - join transfer and consumer threads
   - pump one zero-timeout libusb event pass
5. `cancel_transfers`
   - call `libusb_cancel_transfer` on every transfer
   - pump events with `struct timeval { 0, 50 }` while transfers are live

The C++ driver aggressively cancels pending transfers and wakes the libusb event
loop. The Rust driver currently relies on stop flags plus transfer completion or
timeout.

## nusb Semantics Relevant To This Driver

Local source: `nusb-0.2.4`.

- `Endpoint::cancel_all()` requests asynchronous cancellation of all pending
  transfers. Cancelled transfers are returned later from completion calls.
- `Endpoint::wait_next_complete(timeout)` blocks up to the timeout and returns
  `None` on timeout. It does not cancel the pending transfer.
- `Endpoint::transfer_blocking()` shows the intended cancellation pattern for a
  timeout: submit, wait, `cancel_all()`, then keep waiting until the cancelled
  completion returns.
- `Endpoint::clear_halt()` is documented for stall recovery and should not be
  called while transfers are pending.
- On endpoint drop, nusb backends call `cancel_all()` if transfers are still
  pending, but relying on drop means cancellation timing is implicit and delayed
  until control leaves the producer loop.

Platform notes from local nusb backends:

- macOS: `clear_halt` calls IOKit `clear_pipe_stall_both_ends`.
- Linux: `clear_halt` maps to USBFS `CLEAR_HALT`.
- Windows: `clear_halt` maps to `WinUsb_ResetPipe`.

## libusb Semantics Checked

Official libusb docs:

- `libusb_cancel_transfer()` is asynchronous. Cancellation completion is
  reported later through the normal transfer callback path, and applications
  must not free a cancelled transfer before that completion has arrived.
- On macOS/iOS, libusb cannot cancel one transfer in isolation; cancelling one
  transfer on an endpoint causes all transfers on that endpoint to be cancelled.
  That matches the `cancel_all()` shape used in the Rust patch.
- Cancelled transfers may have partial data; for this driver's stop path, the
  policy is to discard cancellation completions because receiver-off/restart is
  already a stream boundary.
- `libusb_clear_halt()` documentation says pending transfers should be cancelled
  before clearing halt. This supports removing stop-time `clear_halt_ep` and
  making stall recovery cancel/drain before `ep.clear_halt()`.

Sources:

- https://libusb.sourceforge.io/api-1.0/group__libusb__asyncio.html
- https://libusb.sourceforge.io/api-1.0/group__libusb__dev.html

## Leading Hypotheses

### H1: Stop latency was dominated by the 1000 ms producer wait

The producer checks `stop_requested` only before and after `wait_next_complete`.
If receiver-off does not complete or cancel the pending transfer promptly, stop
could wait until timeout. This is the strongest explanation for slow reset and
is the first thing patched.

Experiment:

- Add timestamped tracing around `stop_streaming`:
  - set stop flag
  - receiver off request start/end
  - generation increment
  - producer join start/end
  - consumer join start/end
- Add producer timestamps around `wait_next_complete`.
- Run repeated start/stop cycles and record P50/P95/P99 stop duration.

Patch applied:

- Reduced producer wait timeout from 1000 ms to 10 ms.
- On stop/generation mismatch, producer calls `ep.cancel_all()` and drains
  returned completions briefly before exit.

Follow-up measurement:

- Compare restart pop and stop latency with the client app.
- Measure CPU cost if the app remains streaming for a long session.

### H2: `clear_halt_ep` during stop is out of order

Rust stop does:

1. receiver off
2. generation bump
3. host-side `CLEAR_FEATURE ENDPOINT_HALT`
4. join producer

nusb and libusb both say halt clear should not be called with pending transfers.
The Rust stop path used a control transfer helper rather than the producer's
`Endpoint::clear_halt()`, but it could still race with pending bulk IN
transfers.

Patch applied:

- Removed stop-time `clear_halt_ep`.
- Stall recovery now cancels/drains the endpoint before clearing halt.

Experiment:

- Compare three stop orderings:
  - current: receiver off, clear halt, join
  - C++-like: receiver off, cancel/drain transfers, join, no stop clear-halt
  - hybrid: receiver off, cancel/drain, join, then clear halt before next start
- For each, measure stop duration and first-buffer artifact rate after restart.

### H3: Start sequence may deliver unsettled first buffers

The pop may be from one or more early buffers after receiver-on while analog,
firmware, DC, or DSP state settles. The C++ driver resets `device->vec` and queue
counters at start. Rust initializes consumer rotation state per consumer thread,
but the audio client may still receive first blocks that include hardware
transient content.

Experiment:

- During restart, mark the first N blocks and collect:
  - max absolute sample
  - mean I/Q
  - RMS
  - DC offset before/after IQ balancing
  - dropped sample count
- Compare Rust vs C++ with the same client and sample rate.
- Test dropping or fading the first 1-8 buffers after receiver-on.

Candidate fix to test:

- Add a restart warmup policy internal to the consumer:
  - discard first N blocks after start, or
  - ramp gain over first N blocks.
- Keep callback-visible sample timing in mind. A discard changes latency; a ramp
  preserves callback cadence.

### H4: Thread creation order changes the first-transfer timing

C++ submits transfers before creating the event thread and consumer. Rust creates
consumer first, producer second, and the producer submits transfers inside its
thread after `set_alt_setting` and endpoint construction.

Experiment:

- Measure time from receiver-on to first successful bulk completion.
- Measure time from first bulk completion to first callback.
- Compare Rust and C++.

Possible future design:

- Keep a streaming worker object alive between stops so endpoint setup and buffer
  allocation are not repeated every restart.
- Or split producer initialization so endpoint setup and buffer allocation happen
  before receiver-on.

## Instrumentation Plan

Add a compile-time or env-gated tracing mode. Do not print in the callback hot
path by default.

Suggested env var:

```text
AIRSPYHF_TRACE_STREAM=1
```

Suggested events:

```text
open.done
start.enter
start.receiver_off.done
start.claim.done
start.clear_halt.done
start.receiver_on.done
producer.enter
producer.set_alt.done
producer.endpoint.done
producer.submit_initial.done
producer.complete.ok
producer.complete.stall
producer.complete.cancelled
stop.enter
stop.receiver_off.done
stop.cancel_requested.done
stop.producer_join.done
stop.consumer_join.done
stop.exit
callback.first_block
```

Record monotonic timestamps in microseconds. For callback-facing block metrics,
aggregate in memory and print on stop rather than logging every buffer.

## Success Criteria

- Stop duration under normal conditions is consistently below 50 ms.
- Restart first audible block has no pop in the reference client.
- No increase in dropped samples during steady-state streaming.
- No regression in `cargo test`, Miri/Loom/Kani evidence, and hardware
  `airspyhf_info` parity.

## Patch Log

### Patch 1: explicit producer cancellation on stop

Date: 2026-07-01.

Changes:

- `PRODUCER_WAIT_MS = 10`.
- `PRODUCER_CANCEL_DRAIN_MS = 50`.
- Stop path no longer clears halt while producer transfers may be pending.
- Producer calls `Endpoint::cancel_all()` and drains returned completions on
  stop/generation mismatch.
- Stall path cancels/drains before `ep.clear_halt()`.

Why this patch first:

- It is directly supported by nusb and libusb source/docs.
- It mirrors the C++ driver's explicit `libusb_cancel_transfer` shutdown path.
- It is small enough to evaluate with the real client before adding more
  instrumentation or warmup policy.
