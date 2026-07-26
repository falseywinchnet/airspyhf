# Packing qualification specimens

This directory contains isolated code-generation inputs.  Nothing here is
linked into a firmware image.

Compile the specimen with the qualified firmware compiler:

```sh
TC=../../../.toolchains/arm-gnu-toolchain-14.2.rel1-darwin-arm64-arm-none-eabi/bin
"$TC/arm-none-eabi-gcc" -std=gnu99 -O2 -mcpu=cortex-m4 -mthumb \
  -ffunction-sections -c packing_kernel.c -o packing_kernel-gcc.o
"$TC/arm-none-eabi-objdump" -d packing_kernel-gcc.o
```

The null mover uses volatile accesses intentionally.  It is only a traffic
specimen; an on-device test must reserve every bank it touches from DMA and USB
before it runs.

The native verifier checks the bit contract and in-place overwrite safety:

```sh
cc -O2 -std=c11 packing_kernel.c verify_packing.c -o verify_packing
./verify_packing
```
