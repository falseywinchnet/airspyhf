#include "nibble_plane.h"

#include <limits.h>

#define NPL_MAGIC 0x4eu

static void put_u16(uint8_t *output, uint16_t value)
{
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8);
}

static uint16_t get_u16(const uint8_t *input)
{
    return (uint16_t)input[0] | ((uint16_t)input[1] << 8);
}

static int predict_from_low(uint16_t previous, uint8_t low)
{
    int delta = (int8_t)(uint8_t)(low - (uint8_t)previous);
    return (int)previous + delta;
}

static size_t packed12_size(size_t count)
{
    return (count / 2u) * 3u + (count & 1u ? 2u : 0u);
}

static size_t pack12(const uint16_t *input, size_t count, uint8_t *output)
{
    size_t i;
    size_t cursor = 0;
    for (i = 0; i + 1 < count; i += 2) {
        uint16_t a = input[i] & 0x0fffu;
        uint16_t b = input[i + 1] & 0x0fffu;
        output[cursor++] = (uint8_t)a;
        output[cursor++] = (uint8_t)((a >> 8) | ((b & 0x000fu) << 4));
        output[cursor++] = (uint8_t)(b >> 4);
    }
    if (i < count) {
        uint16_t a = input[i] & 0x0fffu;
        output[cursor++] = (uint8_t)a;
        output[cursor++] = (uint8_t)(a >> 8);
    }
    return cursor;
}

static int unpack12(const uint8_t *input, size_t length,
                    uint16_t *output, size_t count)
{
    size_t i;
    size_t cursor = 0;
    if (length != packed12_size(count)) return 0;
    for (i = 0; i + 1 < count; i += 2) {
        output[i] = (uint16_t)input[cursor] |
                    ((uint16_t)(input[cursor + 1] & 0x0fu) << 8);
        output[i + 1] = ((uint16_t)input[cursor + 1] >> 4) |
                        ((uint16_t)input[cursor + 2] << 4);
        cursor += 3;
    }
    if (i < count) {
        output[i] = (uint16_t)input[cursor] |
                    ((uint16_t)(input[cursor + 1] & 0x0fu) << 8);
    }
    return 1;
}

size_t npl_encode_block(const uint16_t *input, size_t count,
                        uint8_t *output, size_t capacity,
                        npl_block_info *info)
{
    size_t raw_bytes;
    size_t payload;
    size_t cursor;
    size_t i = count;
    size_t run = 0;
    uint16_t previous = 0;
    uint32_t nonzero = 0;
    uint8_t mode = NPL_MODE_PACK12;
    uint8_t *body;

    if (!input || !output || !count || count > UINT16_MAX) return 0;
    raw_bytes = packed12_size(count);
    if (raw_bytes > UINT16_MAX || capacity < NPL_HEADER_SIZE + raw_bytes) return 0;
    body = output + NPL_HEADER_SIZE;

    /*
     * The low-byte plane has a known fixed offset, so build it and its sparse
     * correction tail in one pass.  Stop as soon as the tail can no longer
     * beat literal 12-bit packing; only fallback blocks need a second pass.
     */
    cursor = count + 1u;
    if (cursor < raw_bytes) {
        put_u16(body, input[0] & 0x0fffu);
        previous = input[0] & 0x0fffu;
        for (i = 1; i < count; ++i) {
            uint16_t current = input[i] & 0x0fffu;
            int predicted = predict_from_low(previous, (uint8_t)current);
            body[i + 1u] = (uint8_t)current;
            if ((int)current == predicted) {
                ++run;
            } else {
                int correction = ((int)current - predicted) / 256;
                while (run >= 255u) {
                    if (cursor + 2u >= raw_bytes) goto fallback;
                    body[cursor++] = 255u;
                    body[cursor++] = 0u;
                    run -= 255u;
                }
                if (cursor + 2u >= raw_bytes) goto fallback;
                body[cursor++] = (uint8_t)run;
                body[cursor++] = (uint8_t)(int8_t)correction;
                run = 0;
                ++nonzero;
            }
            previous = current;
        }
        mode = NPL_MODE_LOW8_RLE;
        payload = cursor;
        goto finish;
    }

fallback:
    if (info && i < count) {
        for (; i < count; ++i) {
            uint16_t current = input[i] & 0x0fffu;
            int predicted = predict_from_low(previous, (uint8_t)current);
            nonzero += current != predicted;
            previous = current;
        }
    }
    payload = pack12(input, count, body);

finish:
    output[0] = NPL_MAGIC;
    output[1] = mode;
    put_u16(output + 2, (uint16_t)count);
    put_u16(output + 4, (uint16_t)payload);
    if (info) {
        info->mode = mode;
        info->sample_count = (uint16_t)count;
        info->payload_bytes = (uint16_t)payload;
        info->nonzero_corrections = nonzero;
    }
    return NPL_HEADER_SIZE + payload;
}

size_t npl_decode_block(const uint8_t *input, size_t length,
                        uint16_t *output, size_t capacity,
                        npl_block_info *info)
{
    uint8_t mode;
    size_t count;
    size_t payload;
    const uint8_t *body;
    size_t token;
    size_t scan = 1;
    size_t event = SIZE_MAX;
    int correction = 0;
    uint32_t nonzero = 0;
    size_t i;

    if (!input || !output || length < NPL_HEADER_SIZE || input[0] != NPL_MAGIC)
        return 0;
    mode = input[1];
    count = get_u16(input + 2);
    payload = get_u16(input + 4);
    if (!count || count > capacity || length != NPL_HEADER_SIZE + payload)
        return 0;
    body = input + NPL_HEADER_SIZE;

    if (mode == NPL_MODE_PACK12) {
        if (!unpack12(body, payload, output, count)) return 0;
    } else if (mode == NPL_MODE_LOW8_RLE) {
        size_t low_bytes = count + 1u;
        if (payload < low_bytes) return 0;
        output[0] = get_u16(body) & 0x0fffu;
        token = low_bytes;

#define LOAD_EVENT() do { \
            event = SIZE_MAX; \
            while (token < payload) { \
                uint8_t run_value; \
                int8_t delta; \
                if (payload - token < 2u) return 0; \
                run_value = body[token++]; \
                delta = (int8_t)body[token++]; \
                if (!delta) { \
                    if (run_value != 255u) return 0; \
                    scan += 255u; \
                    continue; \
                } \
                event = scan + run_value; \
                scan = event + 1u; \
                correction = delta; \
                ++nonzero; \
                break; \
            } \
        } while (0)

        LOAD_EVENT();
        for (i = 1; i < count; ++i) {
            uint8_t low = body[i + 1u];
            int value = predict_from_low(output[i - 1u], low);
            if (event == i) {
                value += correction * 256;
                LOAD_EVENT();
            }
            if (value < 0 || value > 4095) return 0;
            output[i] = (uint16_t)value;
        }
        if (event != SIZE_MAX || token != payload) return 0;
#undef LOAD_EVENT
    } else {
        return 0;
    }

    if (info) {
        info->mode = mode;
        info->sample_count = (uint16_t)count;
        info->payload_bytes = (uint16_t)payload;
        info->nonzero_corrections = nonzero;
    }
    return length;
}
