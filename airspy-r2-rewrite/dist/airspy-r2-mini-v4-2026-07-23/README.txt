Airspy R2/Mini v4 firmware and SDR# Wine bridge
Build date: 2026-07-23

CONTENTS

airspy-helper
  Native arm64 macOS bridge helper. Keep it beside libairspy.0.dylib.

libairspy.0.dylib
  Matching native libairspy library required by airspy-helper.

airspy.dll
  x86-64 Windows libairspy shim. Put it beside SDRSharp.exe.

airspy-r2-mini-v4-universal.bin
  Universal Airspy R2/Mini firmware. This is not Airspy HF firmware.

BRIDGE USE

1. Keep airspy-helper and libairspy.0.dylib in the same directory.
2. Put airspy.dll beside SDRSharp.exe.
3. Run airspy-helper on macOS, then start SDR# under Wine.
4. The default loopback port is 53978. Set AIRSPY_BRIDGE_PORT to the same
   value for both processes only if an alternate port is needed.

The helper requires libusb from Homebrew at /opt/homebrew/opt/libusb.

WHAT CHANGED SINCE THE LAST STABLE V3 FIRMWARE

- USB transfer descriptors now use FIFO free-list and active-tail bookkeeping,
  making allocation and queue append constant-time.
- Ten transfer objects settle onto the ten ADC ring buffers predictably.
- Descriptor pointer 0 is rebuilt on every submission because ChipIdea DMA
  advances it; only immutable page pointers 1 through 4 are cached.
- Interrupt masking restores the caller's prior PRIMASK state.
- M0/M4 ring cursors use simple increment-and-wrap paths instead of modulo.
- Production telemetry accounting is compiled out of the hot path.

The wire protocol, sample formats, R2/Mini rate tables, clocks, prepared-start
ordering, ownership guard, and no-cool-park policy remain compatible with V3.
Packing remains available but is not required or enabled by this firmware.

QUALIFICATION

Verified on Airspy R2 serial 35AC63DC2D7D704F:

- Recovery-loader readback matched all 20,012 firmware bytes.
- Three normal-firmware readbacks matched all 20,012 bytes.
- Ten repeated unpacked 10 MS/s starts completed without DMA, ownership,
  ADC FIFO, partial-transfer, USB-error, or backpressure failures.
- A sustained unpacked run retired 36,794 full 16 KiB banks (about 603 MB)
  with the same zero-error result.
- The rebuilt Wine bridge delivered 19,988,480 complex samples in two seconds
  at the R2 10 MS/s setting with zero drops.

The inherited firmware version string still reports the 2016 upstream source
revision; use the SHA-256 in SHA256SUMS to identify this build.
