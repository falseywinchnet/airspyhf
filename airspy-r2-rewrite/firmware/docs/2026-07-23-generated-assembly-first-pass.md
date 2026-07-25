# Generated assembly: first optimization pass

This audit uses the M0 and M4 ELF files that produced the live-qualified
prepared-start R2 image. It is not based on generic Cortex-M4 assumptions.

Compiler:

```text
Arm GNU Toolchain 14.2.Rel1
arm-none-eabi-gcc 14.2.1 20241119
```

Regenerate source-correlated assembly with:

```sh
TC=/Users/developer/airspyhf/.toolchains/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin
FW=/Users/developer/airspyhf/airspy-r2-rewrite/firmware/device

"$TC/arm-none-eabi-objdump" -d -S \
  "$FW/airspy_rom_to_ram/airspy_rom_to_ram.elf" > /tmp/airspy-m4.S
"$TC/arm-none-eabi-objdump" -d -S \
  "$FW/airspy_m0/airspy_m0.elf" > /tmp/airspy-m0.S
"$TC/arm-none-eabi-nm" -S --size-sort \
  "$FW/airspy_rom_to_ram/airspy_rom_to_ram.elf"
"$TC/arm-none-eabi-nm" -S --size-sort \
  "$FW/airspy_m0/airspy_m0.elf"
```

## Architectural reality

The LPC4370 M4 has Thumb-2, single-precision VFP, saturation instructions, and
useful packed/DSP operations such as dual 16-bit arithmetic and MACs. It does
not have NEON or a general vector register file. The unpacked transport path is
mostly ownership, descriptor, and MMIO control work, so conventional SIMD is
not the principal opportunity. Data transforms may benefit from DSP
instructions or carefully written assembly; transport benefits more from
removing scans, divisions, repeated descriptor construction, and long critical
sections.

## Concrete findings in the generated code

### 1. M4 ring modulo is unnecessarily expensive

`dma_isr` computes modulo ten twice per bank. GCC correctly avoids division,
but each expression still becomes a multiply-by-`0xcccccccd`, shifts, adds,
and subtracts. Both indices advance sequentially and can use increment plus a
single compare/reset branch.

This is small but deterministic work in the highest-frequency M4 interrupt.

### 2. Terminal-count clear is a read/modify/write

The source uses:

```c
LPC_GPDMA->INTTCCLEAR |= INTTC0;
```

The generated M4 code performs `ldr`, `orr`, `str` against the clear register.
For a write-one-to-clear register, direct assignment is both clearer and one
MMIO read cheaper:

```c
LPC_GPDMA->INTTCCLEAR = INTTC0;
```

The same rule should be audited across all W1C registers.

### 3. Production ISR pays for all diagnostic telemetry

Every bank performs cycle-counter reads plus total, maximum, and count updates.
These measurements were valuable during qualification, but a production build
should be able to compile them out while retaining fault counters and the
ownership contract.

### 4. M0 scans all ten records on every event

The inlined `adc_stream_submit_ready` loop checks all ten 32-byte ownership
records whenever the M0 wakes, even though production is strictly sequential.
A `next_submit_index` cursor can submit consecutive ready generations and stop
at the first unavailable record. This removes nine common-case record probes
and makes cost proportional to new work rather than ring capacity.

### 5. USB append finds the tail by walking the active list

`usb_transfer_schedule_tagged` traverses `queue->active` through `next` links
for every append. With a deep dTD queue this is O(queue depth) work for every
bank. Store an explicit active-tail pointer and update it on append,
completion, cancellation, and flush.

This is a larger and more relevant optimization than trying to vectorize the
control path.

### 6. Fixed dTD fields are rebuilt for every bank

Each submission recomputes all five page pointers using additions, shifts, and
stores even though every ring buffer has a fixed address and fixed 16 KiB
length. Preinitialize one dTD template per buffer and refresh only the fields
that genuinely change: active/token state, generation/tag, callback ownership,
and linkage.

### 7. Interrupt-disabled region is too broad

The M0 append path disables interrupts before linking the transfer and keeps
them disabled through queue-tail traversal and the ATDTW append procedure.
The helpers also use unconditional `cpsid i` / `cpsie i`, rather than saving
and restoring the previous interrupt mask. Refactoring should:

- preserve and restore PRIMASK;
- keep only the ownership publication/link mutation inside the critical
  section;
- remove the linked-list tail walk;
- bound or move diagnostic ATDTW accounting out of the production critical
  path where the hardware rules permit.

### 8. Runtime packing branch remains in every DMA interrupt

The M4 ISR loads and tests `use_packing` on every completion. If packing remains
an optional legacy mode, selecting separate ISR bodies or a mode-specific
completion function at start time would make the unpacked production path
straight-line.

## Compiler-age conclusion

The historical repository names `arm-none-eabi` and points at the old Launchpad
GCC Arm Embedded distribution, but it does not pin an exact compiler release.
The relevant firmware history is from 2015--2016, so GCC 14.2 is many major
generations newer than the toolchain available when the code was authored.
That does not automatically make the firmware fast: modern GCC generated good
constant-division code and reasonable Thumb-2, but it cannot infer the missing
ring cursor, queue tail, preinitialized descriptor ownership, or narrower
critical sections. Those require source-level representation changes.

## Recommended order

1. Make diagnostic instrumentation compile-time selectable.
2. Replace W1C read/modify/write operations with direct writes.
3. Replace modulo-ten ring advancement with increment/reset.
4. Add sequential M0 submit and retire cursors.
5. Give the USB queue an explicit tail.
6. Preinitialize fixed dTD fields.
7. Narrow and correctly restore interrupt critical sections.
8. Rebuild, diff disassembly, and repeat live 10 MSPS qualification after each
   behavior-preserving step.
