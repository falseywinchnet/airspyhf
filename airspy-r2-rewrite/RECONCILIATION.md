# Joint audit reconciliation

The firmware, driver, and contract audits point to three successive targets.
They should not be collapsed into one “optimized rewrite.”

## Target A — legacy-safe and legible

Firmware retains two 16 KiB banks, one bulk-IN dTD, the present M4/M0 division,
the same descriptors, and the same unframed raw and packed bytes. The driver
becomes readable scalar C++ behind the original C ABI. Both sides acquire
explicit internal state and tests without changing the wire.

This target is deliberately capable of reproducing known weaknesses. It is the
oracle against which repairs are measured.

Exit requires all four combinations:

| Firmware | original C driver | readable C++ driver |
|---|---:|---:|
| stock | baseline | pass |
| readable legacy rewrite | pass | pass |

## Target B — transport-correct

Firmware first removes active retry spinning with explicit ownership. Buffer
and dTD depth then increase together, selected from SRAM/bus measurements. The
driver gains a byte-stream assembler, strong cancellation-and-retirement, and
independent USB/consumer block sizing.

The default remains legacy v1. New observability and framing are negotiated
only after a safe off-stream probe. Device loss and host queue loss become
separate quantities.

This target is more important than a new codec: it makes throughput and loss
measurable and removes wasted device cycles.

## Target C — capacity

Only after Target B, evaluate:

- a bank-aware implementation of the existing lossless packed12 format;
- a bounded blockwise lossless format with packed12/raw fallback;
- fewer memory passes on the device;
- fused scalar/SSE4.1/NEON decoding and conversion on the host;
- independently tuned frame, USB request, in-flight, and callback sizes;
- higher sample rates only with RF, thermal, clock, and loss measurements.

No lossless encoding can promise fewer than 12 bits for arbitrary 12-bit noise.
“Uses less data” therefore means a lower average on measured captures, with
verbatim packed12 fallback when compression does not win.

At the high-speed bulk ceiling used in the existing analysis (about
53.248 MB/s), a 90% engineering target is about 47.9 MB/s:

- legacy 16-bit containers: about 12.0 million complex samples/s;
- fixed lossless packed12: about 16.0 million complex samples/s;
- a new codec: capture-dependent and never allowed to assume compression.

These are transport ceilings, not claims about clean analog/RF bandwidth.

## Contract decisions

### Frozen

- default VID/PID, interface, alternate setting, and endpoints;
- legacy request meanings and quirks;
- raw and packed byte order;
- legacy stream remains headerless;
- old firmware probe failure means legacy fallback;
- old drivers always see legacy behavior by default.

### Draft

- project-private requests `0x80` through `0x85`;
- fixed `ASR2` protocol-info bootstrap;
- length-delimited capability records;
- atomic configuration while receiver is off;
- a sequence-aware framed profile with raw fallback.

Draft numbers and layouts may change together in firmware, C++, and Rust test
branches. Once marked stable, an identifier's interpretation is permanent.

### Still requires joint measurement

- exact capture-buffer and dTD counts;
- actual SRAM regions used for additional buffers;
- frame/header layout and whether scatter/gather avoids copies;
- completion interrupt coalescing;
- codec choice and reset interval;
- maximum proven R2 and Mini sample rates;
- whether any firmware leaf is worth a Rust experiment.

## Synchronization rule

Each independent chain reports changes using one of four labels:

1. internal only;
2. legacy-equivalent;
3. additive negotiated feature;
4. breaking/new major.

The joint review accepts contract changes only with:

- updated C/C++/Rust-facing definitions;
- golden and malformed vectors;
- old-host fallback behavior;
- worst-case memory, bytes, and cycles;
- reset, partial-transfer, and error semantics;
- a cross-version compatibility result.

That lets the contract evolve without allowing either implementation to invent
the other side.
