# AirspyHF+ driver — Rust port

A Rust reimplementation of `libairspyhf`, exposing the **same C ABI** as the
reference C++ driver (`../libairspyhf`) so existing applications link against it
unchanged. It uses [`nusb`](https://crates.io/crates/nusb) (pure-Rust USB, no
libusb dependency) and [`rustfft`](https://crates.io/crates/rustfft) for the I/Q
balancer.

The module layout mirrors the C++ sources:

| Rust module (`libairspyhf/src/`) | C++ source            | Contents                                   |
|----------------------------------|-----------------------|--------------------------------------------|
| `commands.rs`                    | `airspyhf_commands.h` | USB vendor request codes                   |
| `device.rs`                      | `airspyhf.cpp`        | device open/close, control I/O, streaming  |
| `iqbalancer.rs`                  | `iqbalancer.cpp`      | FFT-based automatic I/Q imbalance correction |
| `ffi.rs`                         | `airspyhf.h`          | exported `#[no_mangle] extern "C"` surface  |
| `lib.rs`                         | —                     | shared C types + version constants          |

The `tools/` crate ports the five command-line utilities to native Rust bins:
`airspyhf_lib_version`, `airspyhf_info`, `airspyhf_gpio`, `airspyhf_calibrate`,
`airspyhf_rx`.

## DLL naming (important)

SDR consumers (SDR#, SDRangel, …) dynamically load **`airspyhf.dll`** on
Windows — *not* `libairspyhf.dll`. The crate's `[lib] name` is therefore set to
`airspyhf`, which makes cargo emit the correct file on every platform:

| Platform | Produced shared library |
|----------|-------------------------|
| Windows  | `airspyhf.dll`          |
| Linux    | `libairspyhf.so`        |
| macOS    | `libairspyhf.dylib`     |

(Cargo does not prepend `lib` to DLLs on Windows.) The exported C symbols are
unaffected by the library name, so the ABI is identical to the C++ build. If you
want to install a side-by-side "experimental" DLL, pass
`AIRSPYHF_DLL_NAME=experimentalairspyhf` to `install.sh` (or `-DllName` to
`install.ps1`).

## Building

Requires a stable Rust toolchain (`rustup`, `cargo`).

```sh
cargo build --release          # builds the driver + all five tools
cargo test                     # unit tests (hardware tests no-op without a device)
cargo clippy --all-targets     # lint
./verify.sh                    # full available safety evidence chain
```

Optimization level, LTO and codegen-units are set in the workspace
`[profile.release]`. The build is **portable by default** — the x86_64/aarch64
baselines already include SSE2/NEON, which is all the DSP hot loops need. To tune
for the local CPU (not distributable — will SIGILL on other machines):

```sh
RUSTFLAGS="-C target-cpu=native" cargo build --release
# or, via the install script:
AIRSPYHF_NATIVE=1 ./install.sh
```

## Safety and verification

The driver has been hardened so streaming workers never borrow or mutate the
device object. They own their buffers and share only atomics plus
`Mutex<IqBalancer>` state through `Arc<StreamShared>`. The device no longer has
raw owning pointers or blanket `unsafe impl Send/Sync`, raw USB samples are
decoded without alignment-dependent casts, and every exported C ABI entry point
is protected by a panic guard.

The reproducible evidence chain is recorded in
[`VERIFICATION.md`](VERIFICATION.md): normal tests, property tests, Miri, Loom,
and Kani all pass on the current code. `./verify.sh` reruns every available tier
and reports missing optional tools without hiding failures from tools that are
installed.

Follow-up forensic work for restart timing, startup pops, and attenuation
measurements lives under [`docs/`](docs/). Start with
[`docs/FORENSIC_PROJECT_PLAN.md`](docs/FORENSIC_PROJECT_PLAN.md).

## Installing

### macOS / Linux

```sh
./install.sh                       # install into /usr/local (sudo used if needed)
PREFIX=/opt/airspyhf ./install.sh  # custom prefix
```

Installs the shared library into `<prefix>/lib`, the tools into `<prefix>/bin`,
and the C headers into `<prefix>/include/libairspyhf`. On Linux it also installs
the `52-airspyhf.rules` udev rule and runs `ldconfig`.

### Windows

```powershell
.\install.ps1                        # stages into .\dist\install
.\install.ps1 -Prefix C:\airspyhf    # custom prefix
```

## Cross-compiling

Set `TARGET` to a rustc triple. When the target OS differs from the host OS, the
artifacts are **staged** under `dist/<triple>/{lib,bin,include}` rather than
installed into the system, so one machine can produce binaries for the others:

```sh
# From Linux/macOS, build a Windows package (needs the target std + a cross linker):
rustup target add x86_64-pc-windows-gnu
CARGO_TARGET_X86_64_PC_WINDOWS_GNU_LINKER=x86_64-w64-mingw32-gcc \
    TARGET=x86_64-pc-windows-gnu ./install.sh
# -> dist/x86_64-pc-windows-gnu/lib/airspyhf.dll, bin/*.exe, include/...
```

Cross-linking requires the appropriate toolchain to be installed
(e.g. `mingw-w64` for `*-windows-gnu`, a musl/gnu cross toolchain for Linux
targets).

## Parity with the C++ driver

This port was audited against the reference C++ driver for behavioural parity.
Notable points kept in lockstep:

- **I/Q balancer precision** — phase/amplitude and DC-removal accumulators use
  `f64` (the C++ struct uses `double`); per-sample results are cast to `f32` on
  application, matching the C++ `dfloat()` casts.
- **FFT layout** — `rustfft`'s forward transform is followed by a half-swap
  (fftshift) so the spectrum is centered on bin `FFTBINS/2`, exactly like the
  custom C++ `fft()`. Correlation loop bounds also match (`i < FFTBins/2 - 1`).
- **Streaming** — a USB producer thread feeds a bounded queue
  (`RAW_BUFFER_COUNT = 8`) consumed by a separate worker that runs the callback,
  so a slow callback never stalls USB polling. Dropped buffers are counted and
  `dropped_samples` is reported as `dropped_buffers * sample_count`.

Validated on real hardware (AirspyHF+ / AIRSPY RANGER): `airspyhf_info` output
matches the C++ tool byte-for-byte, and a bounded `airspyhf_rx` capture matches
the C++ capture in size and in per-sample statistics (mean magnitude / RMS).
