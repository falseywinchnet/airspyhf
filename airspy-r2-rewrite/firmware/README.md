# Airspy R2/Mini firmware

This is the C-first redevelopment of the Airspy R2/Mini firmware.

```text
device/     current buildable LPC4370 firmware
model/      host-buildable ownership and USB queue invariants
hardware/   qualified images, recovery material, and host tools
docs/       audits and hardware qualification records
```

Run `make` here to build the device image and execute the model tests. See
[`BUILDING.md`](BUILDING.md). The staged engineering plan is
[`PLAN.md`](PLAN.md). The authoritative immediate handoff, including decisions,
reasons, invariants, and exit criteria, is
[`NEXT_ADVANCEMENT.md`](NEXT_ADVANCEMENT.md).

## Architectural baseline

The upstream split is:

- M4F: clocks, ADCHS, GPDMA capture, and optional in-place packing;
- main M0: USB device, vendor control, tuner/clock/flash, and bulk forwarding;
- subsystem M0: halted and clock-disabled;
- two physical 16 KiB sample banks;
- one bulk-IN dTD with a blocking allocation retry.

The current experimental target has moved beyond that baseline: ten alternating
16 KiB banks, ten linked dTDs, generation-qualified ownership, prepared start,
telemetry, and bounded recovery. Upstream behavior remains the parity oracle,
and every behavioral change must remain separately reviewable.

The executable model covers:

- round-robin capture ownership and explicit backpressure;
- fixed dTD pools with linked head/tail/active identities;
- append without re-priming an active chain;
- bounded transport pumping;
- generation checks, cancellation, and structural validation;
- sample, completion, queue-depth, and discontinuity accounting;
- stock-depth, deeper-chain, cancellation, and stress tests.

See [`docs/usb-queue.md`](docs/usb-queue.md) for what the model proves and
[`docs/hardware-contracts.md`](docs/hardware-contracts.md) for the target memory,
DMA, and USB requirements.

Readable transcription and behavioral improvement remain separate review
units. The current forward plan—including halt-free host-congestion handling
through a private alternating pair—is maintained in [`PLAN.md`](PLAN.md).

## Rust position

Rust is technically possible on the M4F and M0, but is not the first production
target. LPC4370 startup, linker placement, multicore loading, USB, DMA, and
barriers remain target-specific unsafe work. The near-term target is readable
C with host-side protocol and ownership oracles.
