# Rust/nusb port gate

This directory intentionally contains no driver implementation yet.

The Rust port starts only after the C++ driver:

- exports the frozen C ABI;
- streams on R2 and Mini with stock firmware;
- passes control and sample golden vectors;
- has explicit transfer-retirement and generation rules;
- supports callback-initiated stop safely;
- cleanly separates scalar DSP from optimized kernels;
- implements legacy fallback for any negotiated extension.

The port will reuse the tested nusb cancellation-and-drain and buffer-loan
patterns from the local AirspyHF migration, but it will use Airspy One's
interface 0/alt 0, endpoint 0x81, real-u12 sample semantics, and contract
fixtures.
