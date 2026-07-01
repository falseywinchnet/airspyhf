# Attenuator Forensics

Status: investigation plan, no behavior changes yet.
Date: 2026-07-01.

## Problem Statement

During client testing, a slightly higher attenuation setting appeared to
attenuate less than a lower setting. This may be a measurement artifact, a
firmware/hardware quirk, AGC interaction, or a driver mismatch. Treat it as an
open forensic question.

## Current Rust Behavior

`rust-migration/libairspyhf/src/device.rs`:

```rust
pub(crate) fn set_att(&mut self, att: f32) -> io::Result<()> {
    let idx = att_index_for(att, &self.supported_att_steps);
    self.att_index = idx as u8;
    self.vendor_out(AIRSPYHF_SET_ATT, idx as u16, 0, &[])
}
```

`att_index_for`:

```rust
steps.iter().position(|&v| v >= att).unwrap_or(0)
```

`set_hf_att(index)` sends the provided index directly after checking that it is
in range.

## C++ Reference Behavior

`libairspyhf/src/airspyhf.cpp`:

```c
uint16_t att_index = 0;

for (uint32_t i = 0; i < device->supported_att_step_count; i++)
{
    if (device->supported_att_steps[i] >= att)
    {
        att_index = i;
        break;
    }
}

AIRSPYHF_SET_ATT, att_index
```

The Rust `set_att(float)` selection logic intentionally matches C++:

- first supported attenuation step `>= requested att`
- fallback to index 0 if nothing qualifies

So a monotonicity anomaly is unlikely to come from the float-to-index selection
alone unless the supported step table differs, the caller uses `set_hf_att`
directly, or another control path changes gain state.

## Variables That Can Confound Measurement

- HF AGC enabled vs disabled.
- HF AGC threshold.
- HF LNA state.
- Bias tee state and external front-end loading.
- Sample rate architecture, Low-IF vs Zero-IF.
- Frequency and band-dependent front-end behavior.
- Signal generator level stability.
- Strong-signal compression or ADC clipping.
- Client-side AGC, squelch, or demodulator gain.
- Pop/restart transient contaminating the measurement window.
- `filter_gain` changes after sample-rate changes.

## Measurement Protocol

Use a stable RF source or a known quiet/noise measurement setup. Avoid changing
more than one variable per run.

Recommended fixed settings:

- fixed frequency
- fixed sample rate
- `airspyhf_set_hf_agc(0)`
- fixed `airspyhf_set_hf_lna(...)`
- fixed AGC threshold even when AGC is off
- client-side AGC disabled
- discard first 0.5-1.0 seconds after any attenuator or stream restart change

For each attenuation index:

1. Set attenuation by index with `airspyhf_set_hf_att(index)`.
2. Wait for settling.
3. Capture a fixed-duration IQ block.
4. Compute:
   - RMS magnitude
   - peak magnitude
   - mean I and Q
   - clipping count
   - dropped sample count
5. Repeat at least 5 times per index.
6. Run the same sequence with the C++ driver.

Then repeat using `airspyhf_set_att(float)` to check the float-to-index mapping.

## Data Table Template

```text
driver: rust | c++
device serial:
firmware:
sample rate:
is_low_if:
frequency:
signal source:
client AGC:
hf_agc:
hf_lna:
bias_tee:

index, step_db, requested_db, rms_dbfs, peak_dbfs, mean_i, mean_q, clipped, dropped, notes
0,     ...,     ...,          ...,      ...,       ...,    ...,    ...,     ...,     ...
1,     ...,     ...,          ...,      ...,       ...,    ...,    ...,     ...,     ...
```

## Hypotheses

### A1: Hardware or firmware step table is non-monotonic in practice

The firmware may expose nominal steps that do not map linearly to realized gain
on all bands or operating states.

Test:

- Log the step table returned by `AIRSPYHF_GET_ATT_STEPS`.
- Sweep by raw index.
- Compare Rust and C++ against the same hardware, firmware, frequency, and
  signal level.

### A2: AGC or LNA is masking attenuation

A higher attenuation setting may be partially compensated by AGC or another gain
stage.

Test:

- Force AGC off.
- Repeat with LNA states fixed.
- Repeat with a signal level low enough to avoid compression.

### A3: Measurement window includes restart or control transient

If the attenuator is changed near a restart, the pop/settling issue can corrupt
short measurements.

Test:

- Change attenuation while already streaming.
- Discard first N buffers after the control write.
- Compare with stop/change/start sequence.

### A4: API mismatch between attenuation-by-db and attenuation-by-index

The float API maps a requested dB value to the first available step greater than
or equal to it. A client expecting nearest-step or exact-index semantics could
observe surprising behavior.

Test:

- Log requested float, selected index, and selected step.
- Compare client calls against raw index calls.

## Proposed Tooling

Add a small diagnostic command later:

```text
airspyhf_att_sweep --freq <hz> --sample-rate <hz> --seconds <n> --mode index|float
```

Output CSV rows with step table, selected index, and IQ statistics. Keep it
separate from the driver until the anomaly is understood.

## Do Not Change Yet

- Do not invert or reorder attenuation steps.
- Do not change `att_index_for`.
- Do not special-case hardware revisions.

The Rust and C++ selection logic currently match. Any driver change here should
be based on a Rust-vs-C++ sweep that proves the mismatch is in the Rust port, not
the hardware or measurement setup.
