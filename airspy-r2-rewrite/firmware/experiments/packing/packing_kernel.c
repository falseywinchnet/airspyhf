#include <stdint.h>

enum {
  PACKING_INPUT_WORDS = 4096,
  PACKING_OUTPUT_WORDS = 3072,
  PACKING_GROUPS = 1024
};

/*
 * Isolated code-generation specimen for the proposed in-place ring packer.
 * This is not linked into firmware.  Keep the source and destination aliases
 * legal: the production operation is intentionally in-place.
 */
__attribute__((noinline))
void packing_kernel_c(uint32_t* source, uint32_t* destination)
{
  uint32_t* const end = source + PACKING_INPUT_WORDS;

  while (source != end)
  {
    const uint32_t input0 = source[0];
    const uint32_t input1 = source[1];
    const uint32_t input2 = source[2];
    const uint32_t input3 = source[3];

    const uint32_t sample0 = input0 & 0xffffu;
    const uint32_t sample1 = input0 >> 16;
    const uint32_t sample2 = input1 & 0xffffu;
    const uint32_t sample3 = input1 >> 16;
    const uint32_t sample4 = input2 & 0xffffu;
    const uint32_t sample5 = input2 >> 16;
    const uint32_t sample6 = input3 & 0xffffu;
    const uint32_t sample7 = input3 >> 16;

    destination[0] =
      (sample0 << 20) | (sample1 << 8) | (sample2 >> 4);
    destination[1] =
      (sample2 << 28) | (sample3 << 16) | (sample4 << 4) | (sample5 >> 8);
    destination[2] =
      (sample5 << 24) | (sample6 << 12) | sample7;

    source += 4;
    destination += 3;
  }
}

/*
 * Phase-zero traffic specimen.  Volatile accesses prevent the compiler from
 * deleting a same-value store.  A real on-device experiment must reserve the
 * source and destination banks from both DMA and USB ownership first.
 */
__attribute__((noinline))
void packing_null_mover(
  volatile uint32_t* source,
  volatile uint32_t* destination)
{
  volatile uint32_t* const end = source + PACKING_INPUT_WORDS;

  while (source != end)
  {
    const uint32_t input0 = source[0];
    const uint32_t input1 = source[1];
    const uint32_t input2 = source[2];
    const uint32_t input3 = source[3];

    destination[0] = input0;
    destination[1] = input1;
    destination[2] = input2;
    (void)input3;

    source += 4;
    destination += 3;
  }
}
