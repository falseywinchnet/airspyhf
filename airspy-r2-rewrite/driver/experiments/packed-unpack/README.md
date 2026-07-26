# Packed-sample host benchmark

This benchmark measures the readable driver's current two-pass packed-sample
path with the production 147456-byte transfer geometry. Eight buffer sets keep
the working set larger than a single transfer and resemble the driver's rotating
USB buffers.

It reports the isolated 12-bit unpack rate and the existing unpack-then-convert
rates for signed 16-bit and float output. It is a diagnostic benchmark, not a
claim about end-to-end USB or IQ-converter throughput.

Build and run on macOS:

```sh
clang++ -O3 -DNDEBUG -std=c++20 \
  -I../../model/include \
  benchmark.cpp \
  neon_unpack.cpp \
  ../../model/src/legacy_unpack.cpp \
  ../../model/src/legacy_unpack_x86.cpp \
  ../../model/src/sample_conversion.cpp \
  -o packed-unpack-benchmark
./packed-unpack-benchmark
```

The x86 source is empty on AArch64. For a Windows x64 build it supplies the
runtime-dispatched SSE4.1 and AVX2 kernels.
