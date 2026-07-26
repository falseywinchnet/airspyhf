#include <stdint.h>
#include <stdio.h>

enum {
  INPUT_WORDS = 4096,
  OUTPUT_WORDS = 3072,
  SAMPLE_COUNT = 8192
};

void packing_kernel_c(uint32_t* source, uint32_t* destination);

static uint32_t pseudo_random_state = 0x41535059u;

static uint16_t next_sample(void)
{
  pseudo_random_state =
    pseudo_random_state * 1664525u + 1013904223u;
  return (uint16_t)((pseudo_random_state >> 8) & 0x0fffu);
}

static void unpack(
  const uint32_t* input,
  uint16_t* output)
{
  for (uint32_t input_index = 0, output_index = 0;
       output_index < SAMPLE_COUNT;
       input_index += 3, output_index += 8)
  {
    output[output_index + 0] = (input[input_index] >> 20) & 0xfffu;
    output[output_index + 1] = (input[input_index] >> 8) & 0xfffu;
    output[output_index + 2] =
      ((input[input_index] & 0xffu) << 4)
      | ((input[input_index + 1] >> 28) & 0xfu);
    output[output_index + 3] =
      (input[input_index + 1] >> 16) & 0xfffu;
    output[output_index + 4] =
      (input[input_index + 1] >> 4) & 0xfffu;
    output[output_index + 5] =
      ((input[input_index + 1] & 0xfu) << 8)
      | ((input[input_index + 2] >> 24) & 0xffu);
    output[output_index + 6] =
      (input[input_index + 2] >> 12) & 0xfffu;
    output[output_index + 7] = input[input_index + 2] & 0xfffu;
  }
}

int main(void)
{
  uint32_t bank[INPUT_WORDS];
  uint16_t expected[SAMPLE_COUNT];
  uint16_t actual[SAMPLE_COUNT];

  for (uint32_t index = 0; index < SAMPLE_COUNT; index += 2)
  {
    expected[index] = next_sample();
    expected[index + 1] = next_sample();
    bank[index / 2] =
      (uint32_t)expected[index]
      | ((uint32_t)expected[index + 1] << 16);
  }

  packing_kernel_c(bank, bank);
  unpack(bank, actual);

  for (uint32_t index = 0; index < SAMPLE_COUNT; ++index)
  {
    if (actual[index] != expected[index])
    {
      fprintf(
        stderr,
        "sample %u: expected %u, decoded %u\n",
        index,
        expected[index],
        actual[index]);
      return 1;
    }
  }

  printf(
    "pass: %u samples, %u input bytes -> %u packed bytes\n",
    SAMPLE_COUNT,
    INPUT_WORDS * 4u,
    OUTPUT_WORDS * 4u);
  return 0;
}
