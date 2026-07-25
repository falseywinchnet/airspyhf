# Independent workstream briefs

These briefs keep future sessions sequential within a project and parallel
between projects.

## Firmware session chain

Scope: `firmware/` and firmware-facing generated contract definitions.

Start with F0/F1 evidence. Then transcribe one subsystem at a time:

1. build/link/startup and board configuration;
2. legacy shared words and M0/M4 events;
3. control dispatcher and exact request traces;
4. ADCHS/GPDMA definitions and generated manifest;
5. USB device/controller and one-dTD legacy queue;
6. raw and packed sample path.

Do not increase depth, remove spinning, move work between cores, change
clocks/interrupts, or change stream bytes during readable parity. Report every
cross-boundary proposal to the joint review rather than implementing it
unilaterally.

## Driver session chain

Scope: `driver/` and host-facing generated contract definitions.

Start with H0/H1 evidence. Then build:

1. exact public C ABI shell;
2. typed legacy control requests over a fake backend;
3. device discovery/open/close;
4. explicit stream lifecycle and transfer retirement;
5. bounded producer/consumer buffer ownership;
6. scalar unpack/conversion/IQ pipeline;
7. stock-firmware hardware parity.

No SIMD, fused kernels, lock-free queue, nusb port, or v2 stream until scalar
C++ parity. Preserve exception-free C ABI boundaries.

## Contract and joint-review chain

Scope: `contract/`, vectors, captures, and this reconciliation.

1. Freeze descriptors and every request transcript.
2. Freeze legacy sample vectors at arbitrary host chunk boundaries.
3. Add deterministic generated definitions for C, C++, and Rust.
4. Run stock/readable cross-compatibility.
5. Review both optimization audits together.
6. Assign experimental IDs and budgets.
7. Admit a feature only after both sides and fallback pass.

The joint review owns the target. Firmware owns device implementation; the
driver owns host implementation; neither owns wire semantics.
