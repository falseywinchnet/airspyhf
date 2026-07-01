# AirSpy HF Wine bridge

Run a Windows AirSpy app (e.g. **SDR#**, 64-bit .NET, under Wine on macOS) against
a **real** AirSpy HF device, without any Wine USB support.

Wine has no usable USB passthrough. Instead of fighting that, we keep all USB on
the native side:

```
SDR# (.NET 10, x86_64 Wine via Rosetta)
   |  P/Invoke (cdecl)  -> airspyhf.dll  (PE x86_64, this bridge's "shim")
   |  TCP 127.0.0.1:53977
airspyhf-helper  (native arm64) -> libairspyhf -> libusb -> IOKit -> AirSpy HF
```

- **`airspyhf.dll`** (shim) exposes the exact public `airspyhf_*` ABI and forwards
  every call over loopback TCP. Two connections per device: a synchronous control
  channel and a one-way data channel that streams IQ frames. The IQ callback into
  the Windows app runs on a Win32 `CreateThread` thread, so it is Wine-managed.
- **`airspyhf-helper`** links the real (native) `libairspyhf` and owns the device.

Status: **proven end-to-end** — open / enumerate / firmware read / stream
(~753 kS/s) / stop / close, driving a live AIRSPY HF from a PE
DLL under Wine.

## Build

Requires `x86_64-w64-mingw32-gcc` (mingw-w64) and the native `libairspyhf`
installed (`pkg-config --exists libairspyhf`).

```sh
./build.sh        # -> out/airspyhf-helper  and  out/airspyhf.dll
```

## Use with SDR#

1. Start the helper (leave it running while you use SDR#):
   ```sh
   ./out/airspyhf-helper
   ```
2. Put **`out/airspyhf.dll`** next to SDR#'s executables inside the Wine prefix
   (the same folder that holds `SDRSharp.exe`), replacing/over-riding the stock
   `airspyhf.dll`.
3. Launch SDR# under Wine and pick the AIRSPY HF+ source.

The optional env var `AIRSPYHF_BRIDGE_PORT` overrides the TCP port (default
53977); set it identically for both the helper and the Wine process.

## Notes / limitations

- One helper serves multiple devices (up to `MAX_DEVICES`), one device per
  control connection.
- `airspyhf_open_fd` returns `AIRSPYHF_UNSUPPORTED` (a host fd can't cross the
  Wine/native boundary; desktop SDR# doesn't use it).
- `airspyhf_get_samplerates(dev, buf, 0)` returns the count in `buf[0]` exactly
  like the real library (it is *not* the function's return value).
- Frames are little-endian raw `airspyhf_complex_float_t`; both ends are LE.
- This is a development bridge, not a security boundary: it listens on loopback
  only.
```
