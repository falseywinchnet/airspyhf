#include "nibble_plane.h"

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#define BLOCK_SAMPLES 4096u
#define BENCH_ROUNDS 8u

static uint32_t prng(uint32_t *state)
{
    *state = *state * 1664525u + 1013904223u;
    return *state;
}

static double now_seconds(void)
{
    struct timespec value;
    clock_gettime(CLOCK_MONOTONIC, &value);
    return value.tv_sec + value.tv_nsec * 1e-9;
}

static int self_test(void)
{
    uint16_t *input = (uint16_t *)malloc(BLOCK_SAMPLES * sizeof(*input));
    uint16_t *output = (uint16_t *)malloc(BLOCK_SAMPLES * sizeof(*output));
    uint8_t *encoded = (uint8_t *)malloc(BLOCK_SAMPLES * 2u + 64u);
    uint32_t state = 0x4e504c31u;
    size_t count;
    size_t i;
    if (!input || !output || !encoded) return 0;

    for (count = 1; count <= BLOCK_SAMPLES;
         count = count < 512u ? count + 1u : count * 2u) {
        size_t written;
        for (i = 0; i < count; ++i) {
            switch (count & 3u) {
            case 0: input[i] = 2048u; break;
            case 1: input[i] = (uint16_t)(i & 0x0fffu); break;
            case 2: input[i] = (uint16_t)((i & 1u) ? 4095u : 0u); break;
            default: input[i] = (uint16_t)(prng(&state) >> 20); break;
            }
        }
        written = npl_encode_block(input, count, encoded,
                                   BLOCK_SAMPLES * 2u + 64u, NULL);
        if (!written ||
            !npl_decode_block(encoded, written, output, count, NULL) ||
            memcmp(input, output, count * sizeof(*input)) != 0) {
            fprintf(stderr, "nibble-plane self-test failed at count %zu\n", count);
            free(input);
            free(output);
            free(encoded);
            return 0;
        }
        if (count == BLOCK_SAMPLES) break;
    }
    free(input);
    free(output);
    free(encoded);
    return 1;
}

static int analyze_file(const char *path)
{
    FILE *file = fopen(path, "rb");
    long byte_length;
    size_t count;
    uint16_t *samples;
    uint16_t *decoded;
    uint8_t *encoded;
    size_t block;
    size_t total = 0;
    size_t rle_blocks = 0;
    uint64_t corrections = 0;
    size_t round;
    volatile size_t sink = 0;
    double begin;
    double elapsed;

    if (!file) {
        perror(path);
        return 1;
    }
    fseek(file, 0, SEEK_END);
    byte_length = ftell(file);
    rewind(file);
    if (byte_length <= 0 || (byte_length & 1)) {
        fprintf(stderr, "%s: not a u16le capture\n", path);
        fclose(file);
        return 1;
    }
    count = (size_t)byte_length / 2u;
    samples = (uint16_t *)malloc(count * sizeof(*samples));
    decoded = (uint16_t *)malloc(BLOCK_SAMPLES * sizeof(*decoded));
    encoded = (uint8_t *)malloc(BLOCK_SAMPLES * 2u + 64u);
    if (!samples || !decoded || !encoded ||
        fread(samples, sizeof(*samples), count, file) != count) {
        fprintf(stderr, "%s: allocation/read failed\n", path);
        fclose(file);
        free(samples);
        free(decoded);
        free(encoded);
        return 1;
    }
    fclose(file);
    for (block = 0; block < count; ++block) samples[block] &= 0x0fffu;

    for (block = 0; block < count; block += BLOCK_SAMPLES) {
        size_t n = count - block < BLOCK_SAMPLES ? count - block : BLOCK_SAMPLES;
        npl_block_info info;
        size_t written = npl_encode_block(samples + block, n, encoded,
                                          BLOCK_SAMPLES * 2u + 64u, &info);
        if (!written || !npl_decode_block(encoded, written, decoded, n, NULL) ||
            memcmp(samples + block, decoded, n * sizeof(*decoded)) != 0) {
            fprintf(stderr, "%s: lossless round-trip failed at sample %zu\n",
                    path, block);
            free(samples);
            free(decoded);
            free(encoded);
            return 2;
        }
        total += written;
        rle_blocks += info.mode == NPL_MODE_LOW8_RLE;
        corrections += info.nonzero_corrections;
    }

    begin = now_seconds();
    for (round = 0; round < BENCH_ROUNDS; ++round) {
        for (block = 0; block < count; block += BLOCK_SAMPLES) {
            size_t n = count - block < BLOCK_SAMPLES ? count - block : BLOCK_SAMPLES;
            sink += npl_encode_block(samples + block, n, encoded,
                                     BLOCK_SAMPLES * 2u + 64u, NULL);
        }
    }
    elapsed = now_seconds() - begin;

    printf("%-42s %6.3f b/sample  rle=%4zu/%-4zu  corrections=%6.3f%%  %7.1f MS/s\n",
           strrchr(path, '/') ? strrchr(path, '/') + 1 : path,
           8.0 * (double)total / (double)count,
           rle_blocks, (count + BLOCK_SAMPLES - 1u) / BLOCK_SAMPLES,
           100.0 * (double)corrections / (double)count,
           (double)count * BENCH_ROUNDS / elapsed / 1e6);

    free(samples);
    free(decoded);
    free(encoded);
    return sink == 0;
}

int main(int argc, char **argv)
{
    int result = 0;
    int i;
    if (!self_test()) return 2;
    if (argc < 2) {
        fprintf(stderr, "usage: %s capture.u16le [...]\n", argv[0]);
        return 1;
    }
    for (i = 1; i < argc; ++i) result |= analyze_file(argv[i]);
    return result;
}
