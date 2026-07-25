# Airspy Mini live-capture notebook

Date: 2026-07-23

This is the first live-radio check of the transport and lossless nibble-plane
idea.  It uses an Airspy Mini because it has the tighter thermal and memory
envelope while retaining the important Airspy One architecture.

## Radio and transport

- Hardware: Airspy Mini
- Firmware: `AirSpy MINI v1.0.0-rc10-0-g946184a 2016-09-19`
- Advertised complex rates: 6 and 3 MS/s
- Corresponding real ADC rates: 12 and 6 MS/s
- Antenna: not intended for VHF, but adequate for broadcast FM

The new Wine bridge was tested with a Windows x86_64 program under Wine.  The
program loaded the replacement `airspy.dll`, crossed the loopback bridge,
opened the Mini through native libairspy/libusb, and streamed float IQ at the
6 MS/s setting.  A ten-second run received 60,030,976 complex samples:
6.003098 MS/s end to end, with zero samples reported dropped.

That test validates the shim ABI, device enumeration, tuning, gain, rate
selection, the native callback, framing, and the Wine callback thread.  It is
not a USB-performance improvement: the native helper still uses upstream
libairspy.

## Native SDR++ cross-check

The Apple-silicon SDR++ 1.3.0 nightly was also tested with its bundled
`airspy_source`, native `libairspy`, and `libusb`.  Its OpenGL/ImGui controls
are almost entirely absent from macOS accessibility, but the application has
a useful `--autostart` option and stores source and demodulator choices in
plain JSON.  With those set before launch, it opened the same Mini directly at
6 MS/s with sensitive gain 14 and no front-end decimation.

The result was a live 6 MHz-wide spectrum and waterfall centered at
104.1 MHz, WFM audio, and successful RDS decode of “Mid-MO's News Radio.”
The process used about 30.5% of one host CPU and 134 MiB resident memory during
this check.  A two-second stack sample showed all of the expected live paths:

- the libairspy consumer and libusb transfer threads;
- `iqconverter_float_process` in host libairspy;
- SDR++'s complex stream and WFM demodulator;
- FFTW and NEON VOLK spectrum work; and
- the PortAudio callback running on a CoreAudio I/O thread.

The system's default output was the built-in iMac speakers at 48 kHz.  This is
a native CoreAudio path; neither MME nor Wine audio is involved.  This
cross-check is especially useful because it separates radio/transport behavior
from the Wine bridge and confirms the capture survey with an unrelated host
application.

## Band survey

Four one-second, 6 MS/s complex-float windows cover 88–110 MHz.  The strongest
200 kHz channel-shaped excesses over their nearby floors were:

| Frequency | Excess |
|---:|---:|
| 106.9 MHz | 31.0 dB |
| 107.7 MHz | 29.3 dB |
| 104.1 MHz | 27.2 dB |
| 106.1 MHz | 18.1 dB |
| 99.3 MHz | 16.7 dB |
| 96.1 MHz | 13.0 dB |
| 105.7 MHz | 12.0 dB |
| 101.5 MHz | 11.9 dB |
| 96.7 MHz | 11.5 dB |
| 101.9 MHz | 11.1 dB |
| 91.3 MHz | 10.3 dB |

The four complex captures and seven raw captures are retained under
`analysis/runtime/mini-live/`.  That directory is intentionally ignored by
Git because the corpus is 212 MiB.

## Raw-capture accounting caveat

The upstream 2016 `airspy_rx` tool labels raw samples as 12-bit when converting
`-n` to a byte limit even when device packing is disabled.  The callback then
writes unpacked 16-bit words.  Thus `-n 12000000` produces an 18,000,000-byte
file containing 9,000,000 unpacked samples, not twelve million samples.

All analysis below reads little-endian 16-bit words and masks them to 12 bits.

## Lossless high-nibble prediction

Split every 12-bit ADC word into:

```text
sample = high_nibble << 8 | low_byte
```

The decoder always receives the current low byte.  Given the previous decoded
sample, it chooses the value congruent to that low byte modulo 256 which is
nearest to the previous sample.  The encoder sends the signed difference
between that predicted high nibble and the true high nibble.

This is lossless.  A wrong prediction is represented by a correction; no
sample is approximated or discarded.

| Capture | ADC std. dev. | Nonzero correction | Outside ±1 | Ideal low byte + correction entropy | zlib-1 analysis surrogate | Adaptive result |
|---|---:|---:|---:|---:|---:|---:|
| 104.1 MHz, gain 14 | 24.7 | 0.00% | 0.000% | 8.00 b/sample | 8.04 b/sample | 8.04 |
| 106.9 MHz, gain 0 | 2.3 | 0.00% | 0.000% | 8.00 | 8.03 | 8.03 |
| 106.9 MHz, gain 8 | 4.1 | 0.00% | 0.000% | 8.00 | 8.03 | 8.03 |
| 106.9 MHz, gain 14 | 45.8 | 1.41% | 0.000% | 8.12 | 8.33 | 8.33 |
| 91.3 MHz, gain 14 | 12.8 | 0.33% | 0.000% | 8.03 | 8.08 | 8.08 |
| 99.3 MHz, gain 14 | 11.7 | 0.01% | 0.000% | 8.00 | 8.04 | 8.04 |
| 106.9 MHz, gain 20 | 766.3 | 90.95% | 73.229% | 11.99 | 12.08 | 12.00 |

“zlib-1” is not a proposed firmware encoder.  It is a convenient measurement
of how cheaply the correction stream can be represented after the low bytes
are separated.  “Adaptive” chooses the smaller of that representation and
ordinary 12-bit packing.

## What this establishes

The nibble-plane idea is real, but conditional.

At sane gains in this FM survey, the low byte plus a sparse correction stream
landed near 8.0–8.3 bits per real ADC sample.  Against ordinary 12-bit packing,
that is roughly a one-third reduction.  At gain 20 on the strongest signal,
the waveform traversed many 256-count regions per sample and the advantage
vanished completely.

Therefore a deployable format needs a cheap, bounded block decision:

1. predict the high nibble while emitting low bytes;
2. use a sparse correction mode only when corrections are cheap;
3. fall back to literal 12-bit packing for a high-motion block;
4. state the mode and exact block length in the transport contract;
5. never wait, retry indefinitely, or drop samples to make a block compress.

The important open engineering question is not whether the representation can
compress these captures.  It can.  The question is whether a custom sparse
encoder plus multibuffer USB queueing costs fewer LPC4370 cycles than the bus
time it saves.  The high-gain capture is also a mandatory regression fixture:
any design that assumes the high nibble is usually predictable will lose on
exactly that input unless it has a fast literal fallback.

## Implemented bounded codec

The follow-up `codec/nibble_plane.c` replaces the zlib analysis surrogate with
an actual lossless format.  It emits the low-byte plane and run-length-coded
signed high-nibble corrections, or immediately falls back to literal 12-bit
packing when the sparse form cannot win.  On the same captures it achieved
8.014–8.251 bits/sample at sane gains and 12.012 bits/sample on the gain-20
fallback fixture.

A matched pair at 104.1 MHz and sensitivity gain 14 was then collected at both
Mini firmware rates:

| Firmware path | Output | Nonzero correction |
|---|---:|---:|
| 12 MHz real / 6 MHz complex | 8.094 b/sample | 0.288% |
| 6 MHz real / 3 MHz complex | 8.014 b/sample | 0.000% |

The lower rate retained at least as much nibble predictability on this
station.  Full format and compute results are in `NIBBLE_PLANE_CODEC.md`.
