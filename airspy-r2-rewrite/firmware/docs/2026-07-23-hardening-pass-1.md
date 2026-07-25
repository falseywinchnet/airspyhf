# Airspy R2/Mini firmware hardening, pass 1

Date: 2026-07-23

Status: source complete and compile-checked with Arm GNU Toolchain 14.2.1. No
device access, flash operation, or runtime qualification was performed. The
resulting build is not a replacement for the live-qualified V4 image.

Superseded behavior note: pass 2 replaces the deliberately fail-stopped GPDMA
policy below with a bounded drop-and-restart handshake. See
`2026-07-23-hardening-pass-2-recoverable-streaming.md`. This file remains the
historical record of the first pass, not a description of current source.

## Provenance of the GPDMA error defect

The production GPDMA interrupt defect is inherited from the 2016 upstream
firmware. Upstream `dma_isr()` reads and clears `INTERRSTAT` only under
`DMA_ISR_DEBUG`, while that option is disabled in ordinary builds.

V3 retained the upstream handler while introducing the guarded ADC ring. V4
changed queue allocation, append cost, descriptor preparation, and ring cursor
cost; it did not change the GPDMA error branch. The defect is therefore present
in vanilla, V3, and the live-qualified V4 image. It was not introduced by the
V4 queue work.

## Corrections in this pass

### Production GPDMA fail-stop

The interrupt handler now clears error status in every build. A channel 0 error:

1. acknowledges both error and any coincident terminal-count interrupt;
2. asserts channel halt;
3. records the DMA error and halted capture;
4. publishes a distinct capture-fault mode;
5. wakes M0.

M0 responds by taking the receiver through OFF, stopping ADC formally and then
cancelling every queued USB owner. The ordinary backpressure-resume service is
forbidden from clearing a fault halt. A later host start constructs a fresh
capture epoch.

This deliberately reports a stream failure instead of attempting to continue
after a possibly corrupt or discontinuous DMA transaction.

### Atomic cross-core command publication

The three M0-to-M4 mailboxes are four-byte objects at aligned linker addresses.
M0 now constructs command and configuration together and publishes them with
one aligned 32-bit store. M4 reads one 32-bit snapshot and extracts both fields.
Compile-time type checking prevents the mailbox from silently growing beyond
one word.

Memory barriers provide release/acquire ordering around command publication and
acknowledgement. This removes the possibility of M4 pairing a new command byte
with the preceding configuration byte.

The acknowledgement waits are still unbounded. Adding a deadline requires a
defined whole-device recovery action and is deferred to the next pass.

### Stale retirement cannot release new ownership

If a USB retirement callback's generation does not match the record's current
submitted generation, it now records an overwrite-risk error and leaves the
bank owned. It no longer writes the stale generation into
`retired_generation`. The ring will halt rather than permit a false-free bank.

### EP0 transaction recovery

A new SETUP packet is treated as the mandatory abort boundary for the prior
control transfer. Both old EP0 directions are retired before the new request is
handled. Stale completion flags from the aborted transaction are cleared so
they cannot be interpreted as a data/status stage of the new request.

On normal completion, the completed control dTD is returned to the pool before
the handler allocates the next stage. EP0 allocation no longer spins inside the
USB ISR; allocation or descriptor-validation failure stalls the control
endpoint so a subsequent SETUP can recover it.

### dTD bounds validation

Scheduling rejects:

- a nonzero transfer with a null buffer;
- byte counts beyond the dTD's 15-bit field;
- a transfer extending beyond its five-page address window.

The two Microsoft OS descriptor requests are clamped to their actual 40-byte
and 142-byte object sizes instead of trusting the host's requested length.

### Suspend policy: preserve, then expose discontinuity

Suspend does not flush queued banks, reset data toggles, or pretend that
anything retired. Existing dTD ownership remains intact.

If suspension consumes all free banks, the established guard halts DMA before
overwrite. On resume the preserved banks drain in order; if the ring had
halted, a suspend-discontinuity counter is incremented. ADC FIFO overflow
accounting records samples lost during the gap.

The shared contract now exposes:

- current suspended state;
- suspend count;
- resume count;
- suspend events that exhausted the ring.

### Configuration-object type correctness

The upstream source declared the Mini and R2 configuration objects with a
smaller type than their definitions. Link-time optimization warned that this
could be misoptimized. Typed accessor functions now return the common
configuration prefix without incompatible external declarations.

### Bounded firmware version construction

The version reply no longer uses unbounded `strcpy()` calls on a linker-mapped
configuration object. Both the fixed hardware name and firmware suffix are
copied within explicit source and destination bounds.

## Compile result

The universal source builds successfully with the recorded qualified compiler:

```text
Arm GNU Toolchain 14.2.1
M0: text 11064, data 712, bss 4096
M4 image: text 19680, data 860, bss 400
M0 binary SHA-256:
aa89a978f9b89a763b2d5d52a995201ef8b1ed0b6b8b3f0a1e5031f97fe6d5e6
Unqualified binary SHA-256:
b1239492d439db7e5218141cb82dd02944656f516fad08704b99197bfc3c53f6
```

The binary remains only in the build directory and was not copied into
`hardware/images`. The remaining build warning is inherited: the M0 ELF has a
load segment with read/write/execute permissions. The pass removed the unused
reset-handler argument warning, corrected the descriptor-string const type,
and corrected the dormant packing routine's `always_inline` declaration.

The pass also found that an incremental M0 rebuild did not force the outer M4
image to re-embed the changed M0 payload: `.incbin` dependencies were invisible
to the compiler-generated dependency files. The ROM-image makefile now names
both embedded binaries as explicit prerequisites. Release builds should still
use the documented full `make clean && make` sequence.

## Deferred to pass 2

1. Bounded USB prime/flush/reset/tripwire waits with one coherent controller
   recovery state machine.
2. Bounded M0-to-M4 command acknowledgement with a whole-device recovery
   action.
3. Bounded PLL, ADCHS, I2C, and flash waits with propagated errors.
4. Production fault persistence and watchdog policy.

Those are not safe as isolated loop-count edits. A timeout must first revoke the
producer, preserve or cancel ownership exactly once, reset the responsible
hardware, and leave host and device in a mutually understandable state.
