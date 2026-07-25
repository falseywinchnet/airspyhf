# Firmware hardware contracts

## Memory

Every target region must eventually be declared once and emitted into a
generated manifest with its address, size, owner, legal bus masters, alignment,
and lifetime.

| Region | Current role |
|---|---|
| `0x10000000`, 128 KiB | M4 code/constants; measured USB test window at `0x10018000`; dTD test metadata at `0x1001c000` |
| `0x10080000`, 72 KiB | M4 data/BSS/stack; measured USB test window at `0x10084000` |
| `0x18000000`, 18 KiB | disabled subsystem-M0 SRAM; not presumed usable |
| `0x20000000`, 16 KiB | stack/shared/config/ADC structures |
| `0x20004000`, 16 KiB | capture bank 0 |
| `0x20008000`, 16 KiB | capture bank 1 |
| `0x2000c000`, 16 KiB | main-M0 executable/data |

Linker assertions must prove image fit, stack separation, sample-bank
alignment, and non-overlap. The USB DMA must never be given bit-band aliases.
Descriptor and payload alignment is a contract, not an incidental attribute.

The unused SRAM question is not answered by summing datasheet capacities. On
2026-07-23, an Airspy Mini running the exact `946184a` firmware base returned
1,000 changing 16 KiB patterns without error from each of `0x10018000` and
`0x10084000` through a USB control-transfer dTD. This proves main-M0 writes and
USB0 DMA reads for those windows on that LPC4370 board. It does **not** yet
prove direct ADCHS/GPDMA writes into those banks. The subsystem-M0 bank at
`0x18000000` remains untested.

The target fixture places four bulk-IN dTD objects in a dedicated 512-byte
section at `0x1001c000`. The M4 linker forbids code from reaching
`0x10018000`; the M0 linker bounds the metadata section to the final 16 KiB of
local SRAM1.

## GPDMA descriptors

Each LLI must become a typed definition containing:

- source and destination regions;
- transfer width and count;
- source/destination increment;
- burst policy;
- terminal-count interrupt policy;
- next LLI identity;
- affected buffer and byte range.

The same definitions should generate target initializers and a readable build
manifest. Checks must prove 16-byte LLI alignment, legal counts, region bounds,
a closed intended chain, and no target concurrently owned by USB.

## USB dTDs

The stock ChipIdea queue implementation has append machinery, but the bulk-IN
pool has one member. The target fixture raises that pool to four only on the
hardware-test branch. The host-buildable model names free, linked, and active
dTD states; connects each live dTD to one generation-qualified buffer; and
tests logical append, automatic chain advance, saturation, cancellation, and
retirement.

No dTD storage may be reclaimed or reused until the controller has retired it.
A buffer cannot return to DMA ownership until every dTD referencing it retires.

The 2026-07-23 target fixture exercises the AN3631/UM10503 sequence on silicon:
publish the tail link, inspect `ENDPTPRIME`, use ATDTW to sample `ENDPTSTAT`,
and re-prime if the controller consumed the old tail. A 1,000-batch active test
produced 3,000 active exits. A separate pending-host-transfer fixture forced
101 endpoint-idle races and observed 101 re-prime exits. All 4,404 dTDs retired
with exact payloads, no tripwire retries, no scheduling failures, and no
controller errors.

This proves the current MMIO algorithm under the generated interleavings. It
does not yet prove sustained ADC capture, cache/barrier policy under every core,
or recovery from injected transaction and buffer errors.

## Endpoint data-toggle lifecycle

USB configuration and application streaming are different boundaries.
`SET_CONFIGURATION`, `SET_INTERFACE`, bus reset, and endpoint-halt recovery may
reset the bulk endpoint data toggle to DATA0. A receiver pause/resume vendor
command may not reset only the device side while the host preserves its toggle.

The stock code called the same endpoint initializer at receiver start. The
hardware fixture reproduced the resulting silent loss: after an odd packet
count, five of five following restarts delivered 65,024 of 65,536 bytes even
though all four dTDs reported successful completion. The first 512-byte packet
was ACKed as a duplicate and discarded by the host.

The target refactor now exposes two explicit operations:

- `usb_endpoint_init`: configure at a USB boundary and reset to DATA0;
- `usb_endpoint_resume`: configure after an application pause and preserve the
  shared host/device toggle.

Ten odd-packet restart cycles passed after the split, followed by a 1,000/101
batch stress test and a new-process continuation.

## Inter-core mailbox

The current wrapping offsets and command words are legacy ABI. First name and
reproduce them. A later fixed-layout mailbox must contain command and
acknowledgement sequences, opcode, arguments, result, and status. Each field has
one writer and one reader.

`volatile` is not a memory-ordering protocol. Publishing and consuming across
cores requires documented compiler and hardware barriers.
