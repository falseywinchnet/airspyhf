# Driver refactor plan

The current concrete implementation handoff is
[`NEXT_ADVANCEMENT.md`](NEXT_ADVANCEMENT.md). Where this long-range plan is
less specific, that document controls the next advancement.

## Objective

Keep the published libairspy C API/ABI and legacy bulk stream intact while
making the implementation readable enough to audit and tighten. The field
behavior is the initial specification; a new implementation does not get to
invent intent where the old code was deliberately conservative.

## Stage 1 — readable, defensible C++

1. Freeze exported symbols, structures, enums, callback semantics, control
   requests, sample formats, error codes, and supported-platform behavior.
2. Wrap the current libusb behavior behind explicit device, transfer-pool,
   consumer-queue, lifecycle, conversion, and public-C-boundary modules.
3. Preserve the proven transport geometry: sixteen 256 KiB asynchronous bulk
   requests plus eight consumer replacement buffers.
4. Preserve immediate pointer-swap resubmission and keep USB completion work
   separate from conversion and application callbacks.
5. Preserve conservative fail-fast behavior when transport ordering or endpoint
   epoch is ambiguous. Classify errors more precisely; do not promise universal
   recovery.
6. Replace volatile cross-thread flags with defined synchronization, make
   cancellation asynchronous but fully drained, and prove that no transfer or
   callback outlives its storage.
7. Preserve full-transfer and host-queue drop accounting, then incorporate
   firmware counters only where their meaning can be mapped honestly onto the
   existing `dropped_samples` contract.
8. Dogfood this build on R2 and Mini across Windows, macOS, and Linux before
   changing transfer geometry.

## Stage 2 — measured tuning

Treat 16 KiB as the natural packet/bank quantum, not automatically the host
request size. Test only whole multiples and require contested-host,
start/stop, unplug, latency, CPU, and loss evidence across COTS platforms.
No Mac-only result replaces the field-proven baseline.

## Stage 3 — possible Rust/nusb port

Port the proven C++ state machine rather than redesigning it. nusb must reproduce
separate USB and consumer buffer capacity, continuous pending depth, ordered
retirement, conservative error boundaries, and cancel-and-drain semantics.
Rust ownership is a hardening mechanism; it is not presumed to improve bulk
priority or wire throughput.
