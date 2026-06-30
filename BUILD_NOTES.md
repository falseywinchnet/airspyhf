# Build notes

## macOS

Verified on macOS with AppleClang 15.0.0 and CMake 4.2.3.

Dependencies used:

```sh
brew install cmake pkg-config libusb
```

The Homebrew libusb available on this machine was `1.0.29`, so no local
libusb build was needed. CMake found it through `pkg-config`:

```sh
pkg-config --modversion libusb-1.0
# 1.0.29
```

Build commands:

```sh
cmake -S . -B build-macos
cmake --build build-macos --parallel
```

The command-line tools run from the build tree after the macOS RPATH fix. For
example:

```sh
./build-macos/tools/src/airspyhf_lib_version
```

Local prefix install tested:

```sh
cmake --install build-macos --prefix /Users/quentinkuttenkuler/airspyhf/install-macos
```

The macOS install name/RPATH must support non-`/usr/local` prefixes. Installed
tools now load `@rpath/libairspyhf.0.dylib` and carry
`@loader_path/../lib`, so `install-macos/bin/*` can run against
`install-macos/lib/*` directly.


The reference tools need direct USB access on macOS. Inside the Codex sandbox,
libusb enumeration failed with Darwin `out of resources` plugin setup errors,
while the same tools worked outside the sandbox.

`airspyhf_rx` can pull samples from the Discovery, but modern firmware rejects the
tool's default legacy/manual HF AGC command. Use `-z` to skip those manual
commands:

```sh
./build-macos/tools/src/airspyhf_rx \
  -s {device_string} \
  -r /private/tmp/discovery_768k_8192_samples.iq \
  -f 7.1 \
  -a 768000 \
  -n 8192 \
  -z
```


## Implementation notes from macOS bring-up

- Updated CMake minimums to work with current CMake releases.
- Enabled C++ for the library targets. The active library sources are
  `airspyhf.cpp` and `iqbalancer.cpp`, not the old `.c` names.
- Set C++11 on the library targets for the existing C++ source constructs.
- Fixed AppleClang/C++ handling of the `RESTRICT` macro.
- Fixed a malformed `VECTORIZE_LOOP` macro in `iqbalancer.cpp`.
- Disabled Clang vectorization pragmas that produced requested-transform
  warnings on AppleClang; GCC/MSVC hints remain in place.
- Fixed macOS build-tree RPATH handling so tools can load the just-built
  `libairspyhf.0.dylib` without installing first.
- Fixed `FindUSB1.cmake` so it reports the same package name requested by
  `find_package(USB1)`.
- Repaired the malformed `rx_callback()` block in `airspyhf_rx.c` and
  initialized its write path cleanly.
- Removed a double-free path in `airspyhf_info.c`.

## Linux reminder

The usual Linux package baseline is still:

```sh
sudo apt-get install build-essential cmake pkg-config libudev-dev
```

If the distribution libusb is too old, build libusb locally. The previously
used target version was libusb `1.0.27`.
