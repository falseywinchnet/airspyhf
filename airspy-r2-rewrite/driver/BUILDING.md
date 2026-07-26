# Building libairspy

## Prerequisites

- CMake
- a C/C++ compiler
- pkg-config
- libusb 1.0 development files
- pthreads where required by the platform

Typical package names are `libusb-1.0-0-dev` on Debian/Ubuntu and `libusb` on
Homebrew or MSYS2.

## One build

From this directory:

```sh
make
```

This builds:

1. `current/`, the field-compatible libairspy baseline and tools;
2. `readable/`, the transitional C++ build of that parity implementation;
3. `model/`, the readable C++ lifecycle/stream scaffold;
4. the model and ABI tests.

The final verification compares the actual exported symbol set of the C and
C++ libraries. An unreviewed ABI addition or deletion fails the build.

Build products are isolated under `.build/` and ignored by Git.

Useful separate targets:

```sh
make current
make readable
make test
make hardware-smoke
make windows-x64
make clean
```

`hardware-smoke` is intentionally not part of `make`: it opens every attached
Airspy R2/Mini, streams at samplerate index zero, exercises callback stop, forces
host queue drops with a slow consumer, and toggles packed mode. Stop SDR
applications and helpers that own the radios before running it.

`windows-x64` cross-builds the standalone native Windows driver and its runtime
package. See [`windows/README.md`](windows/README.md). This is distinct from the
Wine bridge shim.

The first tightening stage deliberately preserves the public C ABI, the USB
request geometry, sixteen 256 KiB asynchronous transfers, the eight-buffer
consumer queue, immediate resubmission, buffer swapping, and conservative
fail-fast boundaries. Tuning begins only after cross-platform parity.
