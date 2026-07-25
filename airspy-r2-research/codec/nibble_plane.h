#ifndef NIBBLE_PLANE_H
#define NIBBLE_PLANE_H

#include <stddef.h>
#include <stdint.h>

#define NPL_HEADER_SIZE 6u
#define NPL_MODE_PACK12 0u
#define NPL_MODE_LOW8_RLE 1u

typedef struct {
    uint8_t mode;
    uint16_t sample_count;
    uint16_t payload_bytes;
    uint32_t nonzero_corrections;
} npl_block_info;

size_t npl_encode_block(const uint16_t *input, size_t count,
                        uint8_t *output, size_t capacity,
                        npl_block_info *info);

size_t npl_decode_block(const uint8_t *input, size_t length,
                        uint16_t *output, size_t capacity,
                        npl_block_info *info);

#endif
