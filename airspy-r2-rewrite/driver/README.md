# Airspy R2/Mini host driver

This project tightens libairspy without changing its published C API/ABI or
assuming that conservative legacy behavior was accidental.

```text
current/      field-compatible libairspy baseline and tools
readable/     transitional C++ candidate built from the parity source
model/        readable C++ lifecycle and stream scaffold
docs/         rewrite gates and transport notes
wine-bridge/  SDR# compatibility helper
rust/         deliberately deferred Rust/nusb port gate
```

Run `make` here to build the current library/tools and execute the model tests.
See [`BUILDING.md`](BUILDING.md). The staged rewrite is specified in
[`PLAN.md`](PLAN.md). The authoritative immediate handoff, including decisions,
reasons, preserved legacy behavior, and exit criteria, is
[`NEXT_ADVANCEMENT.md`](NEXT_ADVANCEMENT.md).

## Production sequence

1. Freeze the existing C ABI and legacy USB behavior.
2. Implement readable scalar C++ behind that same C ABI.
3. Tighten lifecycle, synchronization, cancellation, transfer ownership, and
   evidence without casually broadening recovery.
4. Dogfood the C++ implementation with the field-proven transfer geometry.
5. Tune only after cross-platform parity.
6. Use the differential-tested baseline AArch64 NEON packed-sample kernel;
   retain scalar fallback and evaluate explicit x86 kernels separately.
7. Possibly port the proven architecture to Rust/nusb.

The Rust port, if justified, is written from the proven C++ state machine and
common golden vectors—not by reinterpreting the old monolithic source.

## Present C++ candidate

- byte-safe packed-12 decoding with a bounded AArch64 NEON path;
- explicit stream lifecycle and generation modeling;
- an explicit sixteen-slot USB submission/cancellation/completion ledger;
- prepared-start and legacy-fallback transactions with complete rollback;
- a modeled sixteen-USB plus eight-consumer zero-copy buffer pool;
- fused packed-to-int16 and packed-to-float conversion, with signed-safe
  scalar fallback matching the legacy numerical result;
- validation/accounting for device status;
- shared contract linkage;
- packing, lifecycle, start-unwind, ownership, stale-status, counter, queue,
  conversion, and discontinuity tests.

`readable/` now connects those owners to actual libusb requests. It has passed
bounded streaming, callback-stop, deliberately slow-consumer, packing, close,
ASan/UBSan, and TSan runs on both the local R2 and Mini. The SDR# Wine helper
now loads the readable release build from its adjacent private dylib, and an
end-to-end Wine capture probe passes. The exact tested scope and remaining
gates are in
[`docs/readable-libusb-integration.md`](docs/readable-libusb-integration.md).

`current/` remains the working driver and parity oracle while the readable
implementation is completed.
Concrete defects and their required dispositions are recorded in
[`docs/legacy-driver-audit.md`](docs/legacy-driver-audit.md).

`readable/` builds the complete parity implementation as C++ and exports the
same symbol set as the C library. Packed decoding, scalar sample conversion,
transfer lifetime, physical buffer ownership, prepared start, and stop/drain
now use independently tested C++ owners. Control wrappers and DSP code remain
in the transitional parity source.

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
