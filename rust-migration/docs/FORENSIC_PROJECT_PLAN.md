# AirspyHF+ 2026 Forensic Improvement Project

Status: planning notes.
Date: 2026-07-01.

The Rust driver is working and tested. The remaining work is refinement:
restart/reset timing, audible startup artifacts, and a suspicious attenuation
observation. This project should move slowly and keep evidence with every patch.

## Documents

- `USB_RESET_FORENSICS.md`: stop/restart timing, nusb behavior, pop hypotheses.
- `ATTENUATOR_FORENSICS.md`: attenuation anomaly hypotheses and measurement plan.

## Ground Rules

- Keep the current known-good driver as the baseline.
- One behavioral change per experiment.
- Every experiment records:
  - commit or patch name
  - device serial and firmware
  - OS and nusb version
  - sample rate, frequency, IF mode
  - exact client or tool used
  - measured latency and IQ statistics
- Do not trade soundness for timing.
- Do not hide startup artifacts by client-side gain unless the driver also
  documents what it is doing.

## Phase 1: Instrument Only

Goal: make the slow reset and pop measurable.

Tasks:

1. Add optional stream tracing behind `AIRSPYHF_TRACE_STREAM=1`.
2. Add timestamp points around start, stop, producer wait, first callback, and
   callback stop request.
3. Add aggregate first-buffer metrics, printed only on stop.
4. Run Rust and C++ back to back with the same client.

Exit criteria:

- We know whether stop latency is usually producer join, receiver-off control
  transfer, consumer join, or first-buffer settling.
- We know whether pop correlates with first block, dropped blocks, DC offset,
  restart timing, or attenuator/gain state.

## Phase 2: USB Cancellation Prototype

Goal: test whether explicit nusb cancellation removes the slow reset.

Candidate patch:

- Add producer control channel.
- On stop, request producer cancellation.
- Producer calls `Endpoint::cancel_all()`.
- Producer drains completions until `pending() == 0`, accepting
  `TransferError::Cancelled`.
- Join producer.
- Avoid stop-time halt clear unless measurements show it is needed.

Compare against:

- current Rust stop path
- C++ libusb stop path

Exit criteria:

- Restart feels responsive.
- Stop latency distribution is materially improved.
- No new stalls, dropped samples, or deadlocks.

## Phase 3: Startup Artifact Handling

Goal: remove or reduce pop without lying about stream state.

Candidate experiments:

- Discard first N blocks after receiver-on.
- Apply a short linear or raised-cosine ramp to first N blocks.
- Reset IQ balancer/DC state more deliberately on receiver-on.
- Compare C++ behavior with identical capture windows.

Exit criteria:

- Pop is gone or reduced below audibility in the target client.
- First-block metrics support the fix.
- Latency impact is known.

## Phase 4: Attenuator Sweep

Goal: determine whether the attenuation observation is hardware, firmware, or
driver behavior.

Candidate tooling:

- `airspyhf_att_sweep` diagnostic binary or a script around `airspyhf_rx`.
- CSV output for raw index sweeps and float API sweeps.
- Fixed AGC/LNA/client gain.

Exit criteria:

- Rust vs C++ attenuation curves are overlaid.
- Any non-monotonic step is reproduced or ruled out.
- If reproduced in both drivers, document as hardware/firmware/client behavior.
- If Rust-only, patch only the proven mismatch.

## Candidate Metrics

Timing:

- stop enter to receiver-off complete
- receiver-off complete to producer joined
- producer joined to consumer joined
- start enter to receiver-on complete
- receiver-on complete to first bulk completion
- first bulk completion to first callback

Signal:

- first block RMS
- first block peak
- first block mean I/Q
- first block clipping count
- RMS after 1 second
- dropped samples

USB:

- number of pending transfers at stop
- completions drained after cancel
- completion statuses: ok, cancelled, stall, disconnected, fault
- clear-halt count and timing

## Open Questions

- Does receiver-off reliably cause pending nusb IN transfers to complete on
  macOS, Linux, and Windows?
- Is stop-time halt clearing useful, harmless, or a race with pending transfers?
- Does `set_alt_setting(1)` need to happen every stream start, or can it be
  treated as interface setup?
- Does the first audible pop occur before or after IQ/DC correction?
- Does attenuation non-monotonicity persist with AGC off and LNA fixed?
