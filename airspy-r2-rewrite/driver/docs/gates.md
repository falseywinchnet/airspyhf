# Driver gates

## H0 — ABI and behavior inventory

Freeze exported symbols, library names, enum values, public layouts, callback
rules, serial selection, request setup packets, and current packed/unpacked
output.

## H1 — Oracle and fixtures

Run the stock driver against recorded replies and deterministic sample streams.
Record control transcripts and callback output. Hardware captures from both R2
and Mini are required.

## H2 — Readable scalar C++

Introduce RAII device ownership, typed controls, an abstract USB backend,
explicit stream state, transfer ownership, a bounded queue, and scalar DSP.
Preserve the public C header and observable output.

## H3 — Lifecycle hardening

Add generation IDs, cancellation-and-drain, self-stop-safe callbacks,
transactional start/open, bounded replies, and defined short-transfer handling.
Every submitted transfer must produce one observed terminal completion before
its request or buffer is destroyed.

## H4 — Host optimization

Differentially test SSE4.1 and NEON against the scalar reference. Favor fused
unpack/conversion and removal of scratch-buffer passes. Tune transfer size,
outstanding depth, consumer depth, and callback block size independently.
Lock-free structures must win measurements; they are not a default goal.

## H5 — Contract extensions

Probe while stopped, fall back on stall, negotiate atomically, and assemble
frames from an arbitrary USB byte stream. USB request boundaries are never
treated as v2 frame boundaries.

## H6 — Rust/nusb parity

Export the same C ABI, consume the same contract and vectors, and pass the same
hardware, sanitizer/model, lifecycle, and sample-output gates.

## Compatibility matrix

| Device | original driver | C++ rewrite | Rust rewrite |
|---|---:|---:|---:|
| stock firmware | baseline | required | required |
| readable firmware, legacy | required | required | required |
| optimized firmware, legacy | required | required | required |
| negotiated v2 | legacy default | required | required |

The converter source has licensing terms distinct from the BSD-style host
driver. Protocol/USB, public ABI, and DSP provenance must remain isolated until
distribution rights for derivative converter work are confirmed.
