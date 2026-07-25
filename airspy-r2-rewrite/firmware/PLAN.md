# Firmware refactor plan

## Objective

Produce a readable, auditable C firmware for Airspy R2 and Mini while
preserving the legacy USB API and raw sample stream. Improvements must remain
separable from transcription so the reason for every behavioral change is
reviewable.

## Stages

1. **Freeze the baseline.** Preserve upstream provenance, compiler, maps,
   known-good binaries, USB descriptors, control traces, and R2/Mini flash
   recovery material.
2. **Readable target layout.** Name clock, ADC, GPDMA, inter-core mailbox, USB
   endpoint, dTD, tuner, flash, and lifecycle responsibilities without changing
   their external behavior.
3. **Typed hardware contracts.** Centralize memory regions, DMA LLIs, ownership
   generations, barriers, endpoint state, and deadlines. Retain the same wire
   protocol.
4. **Prove parity.** Differential host models, hardware start/stop/replug
   matrices, exact flash readback, sample ordering, and counter stability.
5. **Harden known faults.** Correct errata and demonstrated ownership,
   data-toggle, descriptor-pointer, startup-order, and recovery defects. A
   recoverable event may cause a bounded discontinuity; ambiguous ownership
   still fails safely.
6. **Halt-free USB congestion.** Reserve one alternating SRAM pair outside USB
   ownership. Use eight banks for ordinary transport runway, then redirect the
   live GPDMA chain into the private rolling pair when the latency threshold is
   reached. USB recovery must never halt ADC solely because the host is late.
7. **Minimize controller states.** Normal operation is a linked active queue;
   severe congestion uses one bounded detach/flush attempt and an idle endpoint;
   reentry uses one ownership epoch and one prime. Avoid repeated reset, flush,
   append-retry, or data-toggle transitions.
8. **Optimize measured hot paths.** Compare compiler/code placement, reclaim
   aligned SRAM only when linker assertions prove it, and measure cycles,
   arbitration, current, heat, and RF behavior.
9. **Optional packing.** Full-bank, ownership-gated lossless packing only after
   its worst-case cycle and SRAM-slave schedule preserve the capture deadline.

## Release boundary

The experimental firmware is not called hardened until both R2 and Mini pass
lifecycle, fault-injection, contested-host, long-run, and flash-recovery gates.
Counters are evidence, not proof that every failure is recoverable.
