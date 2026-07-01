# Building the Rust driver for Windows

See [`README.md`](README.md) for the full build/install/cross-compile guide and
the important note on DLL naming (the driver installs as `airspyhf.dll`, not
`libairspyhf.dll`).

## Native build on Windows

With the MSVC toolchain (Visual Studio Build Tools with C++ support) installed:

```powershell
cargo build --release
.\install.ps1            # stages driver + tools + headers into .\dist\install
```

`cargo build --release` produces `target\release\airspyhf.dll` plus the five
tool executables.

The build is portable by default (no `-C target-cpu=native`). To tune for the
local CPU, build with `-Native`:

```powershell
.\install.ps1 -Native
```

## Cross-compiling to Windows from Linux/macOS

```sh
rustup target add x86_64-pc-windows-gnu
CARGO_TARGET_X86_64_PC_WINDOWS_GNU_LINKER=x86_64-w64-mingw32-gcc \
    TARGET=x86_64-pc-windows-gnu ./install.sh
```

This stages `dist/x86_64-pc-windows-gnu/{lib,bin,include}` (it does not install
into the host system). Requires the `mingw-w64` cross toolchain. For an MSVC
target (`x86_64-pc-windows-msvc`) use `cargo-xwin` or build on Windows.
