# Verification Evidence

Captured: 2026-07-01 on macOS/aarch64.

This document records the evidence chain for the Rust AirspyHF+ driver hardening
pass. The goal is narrow and explicit: prove the Rust-owned parts of the driver
are free of Rust UB patterns found in the earlier port (cross-thread `&mut`
aliasing, raw owning pointers, misaligned sample casts, and unwinding across the
C ABI), while preserving the C ABI and hardware behavior.

## Results

| Property | Method | Command | Result |
|---|---|---|---|
| Release build succeeds for driver and tools | Rust compiler | `cargo build --release` | PASS |
| Formatting is stable | rustfmt | `cargo fmt --check` | PASS |
| Lints stay clean with unsafe linting enabled | Clippy | `cargo clippy --all-targets -- -D warnings` | PASS |
| Unit/property/hardware-smoke tests | Cargo test | `cargo test -p libairspyhf` | PASS: 21 passed |
| Pure-core UB/leak/alignment checks | Miri | `cargo +nightly miri test -p libairspyhf --lib` | PASS: 10 passed, 11 ignored |
| Stop protocol interleavings | Loom | `RUSTFLAGS="--cfg loom" cargo test -p libairspyhf stop_protocol_retires_workers` | PASS |
| Bounded proof harnesses | Kani | `cargo kani -p libairspyhf` | PASS: 5 harnesses, 0 failures |
| Hardware info parity | Installed C++ tool vs Rust tool | normalized sequential `airspyhf_info` diff | PASS |

## What Is Proven

- Worker threads do not touch `AirspyHfDevice`; they share only
  `Arc<StreamShared>` with atomics plus `Mutex<IqBalancer>`.
- The device no longer uses a raw owning `IqBalancer` pointer. The balancer is
  RAII-owned by `StreamShared`.
- There is no blanket `unsafe impl Send/Sync` for the device.
- Raw USB bytes are decoded with `chunks_exact(2)` and `i16::from_ne_bytes`, so
  there is no alignment-dependent cast.
- Every exported C ABI entry point is panic-guarded, preventing unwinding across
  `extern "C"`.
- The remaining unsafe operations are explicit and documented with `SAFETY`
  comments.
- Kani proves bounded invariants for samplerate resolution, attenuation index
  selection, frequency kHz clamping, optimal IQ correction point clamping, and
  `mu` correction clamping.
- Loom exhaustively checks the modeled stream stop/generation protocol.

## Scope Notes

Miri skips randomized `proptest` cases because this `proptest` version attempts
failure-persistence filesystem lookups (`getcwd`) under Miri isolation. The same
properties run under normal `cargo test`; deterministic pure-core tests still run
under Miri.

Hardware-touching tests are ignored under Miri. They are normal cargo tests and
no-op when no AirspyHF+ device is present.

Kani reports unsupported constructs from platform/library code during compilation,
but the reachable proof harnesses verify successfully.

Out of scope: the physical receiver, firmware, USB stack, OS driver behavior, and
the foreign callback's own memory safety. The Rust driver treats callback/device
handles according to the inherited `libairspyhf` C contract: one thread owns a
device handle at a time, while streaming workers only use shared atomic state and
owned buffers.

## Reproduction

Run the full available chain:

```sh
./verify.sh
```

Individual commands:

```sh
cargo build --release
cargo fmt --check
cargo clippy --all-targets -- -D warnings
cargo test -p libairspyhf
cargo +nightly miri test -p libairspyhf --lib
RUSTFLAGS="--cfg loom" cargo test -p libairspyhf stop_protocol_retires_workers
cargo kani -p libairspyhf
```
