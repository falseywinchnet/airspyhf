# Reproducible firmware toolchain

The live-qualified Airspy Mini and R2 images in this repository were built
with the complete Arm bare-metal toolchain, not Homebrew's compiler-only
package.

## Qualified compiler

```text
Arm GNU Toolchain 14.2.Rel1
arm-none-eabi-gcc 14.2.1 20241119
Host package: arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi
```

Official release:
[`releases/14.2.rel1`](https://gitlab.arm.com/tooling/gnu-toolchains-for-arm/-/tree/releases/14.2.rel1)

Repository-local location:

```text
airspy-r2-rewrite/.toolchains/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi
```

The toolchain must include the bare-metal C headers, Newlib/Newlib Nano,
binutils, linker, and objcopy. `arm-none-eabi-gcc` existing in `PATH` is not
by itself sufficient.

## Known Homebrew trap

The Homebrew `arm-none-eabi-gcc` 16.1.0 package encountered during development
was configured with `--without-headers`. It failed even on `stdint.h` and
`string.h`. Do not silently fall back to it. Check both the selected executable
and a trivial header compile before building release firmware.

```sh
which arm-none-eabi-gcc
arm-none-eabi-gcc --version
printf '#include <stdint.h>\nint main(void){return 0;}\n' |
  arm-none-eabi-gcc -x c -c -o /tmp/airspy-toolchain-check.o -
```

## Clean build

```sh
export PATH=/absolute/path/to/airspy-r2-rewrite/.toolchains/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin:$PATH
cd /absolute/path/to/airspy-r2-rewrite/firmware/device
make clean
make
arm-none-eabi-size airspy_m0/airspy_m0.elf airspy_rom_to_ram/airspy_rom_to_ram.elf
shasum -a 256 airspy_rom_to_ram/airspy_rom_to_ram.bin
```

The preferred entry point is `make` from the firmware directory; the manual
commands above document exactly what that wrapper invokes.

Build products:

```text
airspy_m0/airspy_m0.elf
airspy_rom_to_ram/airspy_rom_to_ram.elf
airspy_rom_to_ram/airspy_rom_to_ram.bin
airspy_rom_to_ram/airspy_rom_to_ram.list
airspy_rom_to_ram/airspy_rom_to_ram.map
```

## Effective target flags

M4:

```text
-std=gnu99 -O2 -g2 -mcpu=cortex-m4 -mthumb
-mfloat-abi=hard -mfpu=fpv4-sp-d16
-flto -ffunction-sections -fdata-sections
```

M0/M0S:

```text
-std=gnu99 -Os -g2 -mcpu=cortex-m0 -mthumb
-flto -ffunction-sections -fdata-sections
```

The images link without an operating system, using `--specs=nano.specs`,
Newlib Nano, `-lc`, and `-lnosys`.

## Optional experimental builds

Pass experimental definitions through `EXTRA_CFLAGS`, for example:

```sh
make clean
make EXTRA_CFLAGS=-DAIRSPY_MINI_PLL1_144
```

Never reuse objects from a differently configured build. Always run
`make clean` when changing compiler versions or build definitions.

Before release, archive the `.bin`, its SHA-256, the compiler version, the
effective flags, and both M0/M4 size reports. Keep the device's verified stock
flash dump beside the qualification record.
