# Packed Airspy R2 samples on the host

Status: promoted into the readable driver on 2026-07-25  
Primary target: Raspberry Pi-class AArch64 with baseline NEON  
Transfer geometry: 147456 packed bytes per libusb transfer

## What the readable driver did before promotion

The active readable driver calls `unpack_legacy_u12()` whenever firmware
packing is enabled and the application did not request raw samples. Every
12-byte packed group becomes eight unsigned 16-bit samples.

The unpacker is portable scalar C++. It reads three little-endian 32-bit words,
extracts eight 12-bit fields, and writes eight 16-bit values. Conversion to
signed 16-bit or float is a second complete pass over that intermediate
buffer. IQ conversion, when requested, happens after both passes.

There were no explicit NEON, SSE, or AVX intrinsics in the prior unpacker.

The readable driver now uses the bounded baseline-AArch64 NEON kernel
documented below. Packed int16 and float requests fuse decoding with numerical
conversion and write directly into the final output buffer. Other
architectures retain the scalar implementation.

## What the compilers actually emit

Apple Clang 17 at `-O3` leaves the AArch64 loop scalar. Each group performs
three word loads, eight scalar bit extractions/combinations, and eight
halfword stores.

The Windows DLL is built by MinGW GCC at `-O2`. That build also performs scalar
extraction, but assembles the eight results into one XMM register and makes one
16-byte store. This is reasonable baseline code.

MinGW GCC at `-O3` is actively undesirable for this source shape. It attempts
to vectorize eight groups using baseline SSE2, generates a very large shuffle
network, uses every nonvolatile XMM register, and expands the function from
roughly 0x120 bytes at `-O2` to roughly 0x9c5 bytes before its epilogue. The
Windows x64 ABI then requires ten XMM save/restore pairs. We should not obtain
the future optimized Windows build merely by changing the whole driver to
`-O3`.

## Traffic at 10 MS/s complex output

The ADC supplies 20 million real 12-bit samples per second. Packing reduces
the USB input to 30 MB/s. The current host paths cause at least:

| Requested representation | Current traffic | Fused traffic | Avoided |
| --- | ---: | ---: | ---: |
| unsigned 16-bit unpack | 30 MB/s read + 40 MB/s write = 70 MB/s | same | none |
| signed 16-bit | 30 read + 40 intermediate write + 40 intermediate read + 40 output write = 150 MB/s | 30 read + 40 write = 70 MB/s | 80 MB/s |
| float | 30 read + 40 intermediate write + 40 intermediate read + 80 output write = 190 MB/s | 30 read + 80 write = 110 MB/s | 80 MB/s |

These are minimum data-array figures. They exclude IQ conversion, application
callbacks, cache-line effects, and libusb metadata. Each additional radio
adds the same traffic.

The absolute bandwidth is not alarming for a desktop. On a Pi, however,
removing the intermediate pass reduces cache pollution, memory-controller
traffic, consumer-thread occupancy, and energy. It also gives the consumer
more scheduling margin before its eight-buffer queue begins to fill.

## A five-instruction AArch64 unpack

The packed layout permits a much cleaner NEON representation than the scalar
source suggests. A byte-table lookup forms eight little-endian 16-bit
candidates. Even lanes need a four-bit right shift; odd lanes need no shift.
Masking then leaves eight valid 12-bit samples.

The measured steady-state AArch64 loop is:

```asm
ldr   q2, [input], #12
tbl   v2.16b, {v2.16b}, indices.16b
ushl  v2.8h, v2.8h, shifts.8h
bic   v2.8h, #0xf000
str   q2, [output]
```

Loop increment and comparison are additional scalar instructions. All data
instructions above are baseline AArch64/NEON; none is Apple-specific.

A 16-byte vector load consumes only 12 new bytes and overlaps four bytes of
the next group. The experiment deliberately stops one group early and decodes
the final group with a bounded scalar tail. It therefore never reads beyond
the caller's span. A production implementation must preserve this rule.

## Fusion

The same shuffled vector can directly become application output:

- signed 16-bit adds a centered/scaled vector operation and stores the result;
- float widens the centered values, performs fixed-point signed-to-float
  conversion with eleven fractional bits, and stores two float vectors.

This removes the shared `unpacked_samples` scratch pass for the common int16
and float representations. The scratch allocation can remain initially for
behavioral parity and scalar fallback, then be made conditional after hardware
validation.

## Local measurements

The benchmark uses the production transfer size and eight rotating buffer
sets. Results on an Apple M3 varied slightly between runs:

| Path | Observed throughput |
| --- | ---: |
| current scalar unsigned-16 unpack | 6.1–7.3 GS/s |
| experimental NEON unsigned-16 unpack | 26.8–27.2 GS/s |
| current unpack then signed-16 conversion | 5.1–5.9 GS/s |
| experimental fused NEON signed-16 | 27.1 GS/s |
| current unpack then float conversion | 4.5–5.2 GS/s |
| experimental fused NEON float | 12.7–13.3 GS/s |

AddressSanitizer and UndefinedBehaviorSanitizer completed the differential
benchmark without reporting an invalid access or undefined operation.

These numbers are not Raspberry Pi performance claims. The M3 cache and
execution engines are much wider. The useful facts are the verified output,
the 4–5x local unpack gain, the 2.5–3x local float-pipeline gain, the reduced
memory traffic, and the compact generic-AArch64 assembly. Pi 4 and Pi 5
measurements remain a required gate.

At the radio's 20 MS/s real-sample rate, even the current M3 scalar routine
uses well below one percent of one core in this isolated benchmark. The Pi
reason to adopt NEON is therefore margin and efficiency, not a claim that
scalar unpacking alone currently prevents capture.

## Explicit x86 implementation

The corresponding x86 primitive is `PSHUFB`, introduced by SSSE3. Pure SSE2
does not have an equivalent byte-table operation, which explains why GCC's
attempted SSE2 vectorization becomes a large shuffle network. Any processor
meeting the project's SSE4-era minimum also has SSSE3.

The readable driver now contains manually specified SSE4.1 and AVX2 kernels.
The DLL remains a baseline x86-64 binary and chooses the implementation once
at runtime. A pre-SSE4 host uses the deliberately non-vectorized compact
scalar loop.

The SSE loop performs one overlapping load, `PSHUFB`, word shift, `PBLENDW`,
mask, and one store per eight samples. The AVX2 loop constructs two independent
12-byte lanes, then performs the same operations on sixteen samples. Signed
integer and float paths fuse unpacking with conversion and never write the
intermediate unsigned array.

The final group uses an exact 12-byte bounded load. A randomized differential
test places that group directly before a protected page and exercises unsigned,
signed, and float output. This catches both over-read and numerical errors.

MinGW's emitted hot loops contain the intended `PSHUFB`/`PBLENDW` sequence
(and `VPSHUFB`/`VPBLENDW` for AVX2), without the old SSE2 shuffle network or
Windows nonvolatile-XMM spill storm. The whole x86 implementation, including
all three representations, both ISAs, tails, and dispatch, is about 2.6 KiB
of text; individual steady loops are a few hundred bytes.

## Production gates

The promotion retained these production gates:

1. add exhaustive known-layout and randomized differential tests;
2. put the final input group against a protected page to prove bounded reads;
3. test every output representation, packing transition, stop/start, and short
   final span;
4. benchmark scalar, NEON, and fused paths on at least Pi 4 and Pi 5;
5. measure sustained CPU, temperature, and dropped-buffer behavior with one
   radio and with two radios contending;
6. select the AArch64 function once per stream, not once per sample group;
7. preserve raw packed delivery and the public API/ABI exactly.

The experimental source and reproducible benchmark live in
`experiments/packed-unpack/`.
