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
2. `model/`, the readable C++ lifecycle/stream scaffold;
3. the model tests.

Build products are isolated under `.build/` and ignored by Git.

Useful separate targets:

```sh
make current
make test
make clean
```

The first tightening stage deliberately preserves the public C ABI, the USB
request geometry, sixteen 256 KiB asynchronous transfers, the eight-buffer
consumer queue, immediate resubmission, buffer swapping, and conservative
fail-fast boundaries. Tuning begins only after cross-platform parity.
