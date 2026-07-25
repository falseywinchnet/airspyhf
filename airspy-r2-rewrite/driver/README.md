# Airspy R2/Mini host driver

This project tightens libairspy without changing its published C API/ABI or
assuming that conservative legacy behavior was accidental.

```text
current/      field-compatible libairspy baseline and tools
model/        readable C++ lifecycle and stream scaffold
docs/         rewrite gates and transport notes
wine-bridge/  SDR# compatibility helper
rust/         deliberately deferred Rust/nusb port gate
```

Run `make` here to build the current library/tools and execute the model tests.
See [`BUILDING.md`](BUILDING.md). The staged rewrite is specified in
[`PLAN.md`](PLAN.md).

## Production sequence

1. Freeze the existing C ABI and legacy USB behavior.
2. Implement readable scalar C++ behind that same C ABI.
3. Tighten lifecycle, synchronization, cancellation, transfer ownership, and
   evidence without casually broadening recovery.
4. Dogfood the C++ implementation with the field-proven transfer geometry.
5. Tune only after cross-platform parity.
6. Add differential-tested SSE4.1 and practical NEON-equivalent kernels.
7. Possibly port the proven architecture to Rust/nusb.

The Rust port, if justified, is written from the proven C++ state machine and
common golden vectors—not by reinterpreting the old monolithic source.

## Present C++ scaffold

- byte-safe legacy packed-12 decoding;
- explicit stream lifecycle and generation modeling;
- validation/accounting for device status;
- shared contract linkage;
- packing, lifecycle, stale-status, counter, queue, and discontinuity tests.

`model/` is not yet a USB-capable replacement. `current/` remains the working
driver and parity oracle while the readable implementation is constructed.

The intended internal boundary is:

```text
public C ABI
  -> exception/argument boundary
  -> device and explicit stream lifecycle
  -> protocol requests and byte-stream handling
  -> abstract USB backend
  -> transfer pool and bounded consumer queue
  -> scalar sample pipeline
  -> optional dispatched SIMD kernels
```

The minimum optimized host class is SSE4.1 on x86 or its practical NEON
equivalent on ARM. Scalar code remains the specification and fallback.
