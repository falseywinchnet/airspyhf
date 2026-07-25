# Building the firmware

## Prerequisites

- GNU Make
- CMake 3.20 or newer for the host-side model
- Arm GNU Toolchain 14.2.Rel1, complete bare-metal distribution

Download the `arm-none-eabi` package for the build host from Arm's
[GNU Toolchain 14.2.Rel1 release](https://gitlab.arm.com/tooling/gnu-toolchains-for-arm/-/tree/releases/14.2.rel1).
Extract it as:

```text
airspy-r2-rewrite/.toolchains/
  arm-gnu-toolchain-14.2.rel1-<host>-arm-none-eabi/
```

On Apple Silicon the qualified directory name is:

```text
arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi
```

The entire `.toolchains/` directory is ignored by Git. A different location
may be supplied explicitly:

```sh
make TOOLCHAIN_ROOT=/absolute/path/to/arm-gnu-toolchain-14.2.rel1-... 
```

Do not use Homebrew's compiler-only package: the encountered package omitted
the bare-metal headers and could not compile `stdint.h`.

## One build

From this directory:

```sh
make
```

This performs the toolchain/header check, builds the device image, builds the
host ownership/queue model, and runs its tests.

The flashable image is:

```text
device/airspy_rom_to_ram/airspy_rom_to_ram.bin
```

Useful separate targets:

```sh
make firmware
make test
make clean
```

Release builds must record the compiler version, M0/M4 size output, image
SHA-256, and the matching qualification record. Qualified historical images
remain under `hardware/images/`; generated objects in `device/` are ignored.

See [`TOOLCHAIN.md`](TOOLCHAIN.md) for effective flags and the manual commands.
