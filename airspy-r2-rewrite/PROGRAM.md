# Program Contract

## Non-negotiable ordering

Readable parity and optimization are separate review units. A patch may either:

1. name/restructure behavior while preserving it, or
2. optimize already-named behavior with evidence.

It must not do both. This makes regressions attributable and keeps surprising
legacy behavior visible instead of accidentally “fixing” it during cleanup.

## Evidence ladder

| Level | Required evidence |
|---|---|
| P0: model | Unit tests for ownership, lifecycle, and byte encoding |
| P1: source parity | Public C ABI and legacy request/format inventory |
| P2: replay parity | Golden USB control traces and captured sample buffers |
| P3: hardware parity | R2 and Mini start/stop/replug/error matrix |
| P4: performance | Cycles, bus utilization, SRAM traffic, loss, temperature |
| P5: endurance | Repeated lifecycle and long streaming runs |

Optimization claims require P3 and P4. A new stream format additionally
requires corruption, incompressible-input, and fallback tests.

## Parallel chains

### Firmware chain

`source inventory -> readable C -> target parity -> internal scheduling work
-> optional negotiated format`

### Driver chain

`ABI inventory -> readable C++ -> hardware parity -> scalar kernels
-> dispatched SSE4.1/NEON kernels -> Rust/nusb parity`

The chains synchronize only through a released contract revision. Draft
features may be implemented on both sides, but are not the default and must
fall back to legacy v1.

## Decisions deferred on purpose

- The final number of capture buffers and dTDs.
- Whether framing belongs in the continuous stream or in USB transfer metadata.
- Which sub-12-bit lossless codec, if any, wins on real RF and noise captures.
- Whether M4 clock changes are thermally and electrically acceptable.
- Whether any firmware component should move to Rust.

Those are measured decisions, not scaffold assumptions.

## Definition of “identical behavior”

For the readable stages it means:

- same exported C names, values, ownership rules, and callback behavior;
- same legacy requests, directions, values, indexes, lengths, and stall cases;
- same default USB descriptors and raw/packed byte interpretation;
- bit-identical conversion output for accepted golden inputs;
- equivalent start, stop, cancel, unplug, and error transitions;
- no new sample-loss policy hidden behind the word “nonblocking.”

Where legacy behavior is undefined, the rewrite must label the ambiguity and
test the chosen behavior rather than pretending it was specified.
