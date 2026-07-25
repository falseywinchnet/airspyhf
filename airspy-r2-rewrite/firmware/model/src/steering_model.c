#include "airspy_firmware/steering_model.h"

#include <stdlib.h>
#include <string.h>

/*
 * UM10503 memory map. Each of these is a separate slave port on the AHB
 * multilayer matrix, which is what makes alternating between them useful.
 */
uint8_t airspy_steering_slave_of(const uint32_t address)
{
    if (address >= 0x10000000u && address < 0x10020000u) {
        return 0; /* 128 kB local SRAM */
    }
    if (address >= 0x10080000u && address < 0x10092000u) {
        return 1; /* 72 kB local SRAM */
    }
    if (address >= 0x20000000u && address < 0x20008000u) {
        return 2; /* 32 kB AHB SRAM */
    }
    if (address >= 0x20008000u && address < 0x2000C000u) {
        return 3; /* 16 kB AHB SRAM */
    }
    return AIRSPY_STEERING_SLAVE_NONE;
}

airspy_steering_result airspy_steering_layout_init(
    airspy_steering_layout* const layout,
    const uint32_t* const addresses,
    const size_t count)
{
    if (layout == NULL || addresses == NULL || count == 0) {
        return AIRSPY_STEERING_BAD_ARGUMENT;
    }
    if (count > AIRSPY_STEERING_MAX_BANKS) {
        return AIRSPY_STEERING_TOO_MANY_BANKS;
    }

    memset(layout, 0, sizeof(*layout));
    layout->bank_count = count;
    for (size_t i = 0; i < count; ++i) {
        const uint8_t slave = airspy_steering_slave_of(addresses[i]);
        if (slave == AIRSPY_STEERING_SLAVE_NONE) {
            return AIRSPY_STEERING_UNMAPPED_ADDRESS;
        }
        layout->addresses[i] = addresses[i];
        layout->slave[i] = slave;
        layout->slave_size[slave]++;
    }
    return AIRSPY_STEERING_OK;
}

int airspy_steering_static_order_ok(const airspy_steering_layout* const layout)
{
    if (layout == NULL || layout->bank_count < 2) {
        return 0;
    }
    for (size_t i = 0; i < layout->bank_count; ++i) {
        const size_t next = (i + 1u) % layout->bank_count;
        if (layout->slave[i] == layout->slave[next]) {
            return 0;
        }
    }
    return 1;
}

/*
 * Abstract state: how many banks on each slave port are not yet submitted, plus
 * which slave the next destination is already committed to. Encoded as a mixed
 * radix index so the visited set is a flat array.
 */
typedef struct {
    size_t radix[AIRSPY_STEERING_SLAVE_COUNT];
    size_t stride[AIRSPY_STEERING_SLAVE_COUNT];
    size_t committed_stride;
    size_t total;
} encoding;

static void encoding_init(encoding* const e,
    const airspy_steering_layout* const layout)
{
    size_t total = 1;
    for (size_t g = 0; g < AIRSPY_STEERING_SLAVE_COUNT; ++g) {
        e->radix[g] = layout->slave_size[g] + 1u;
        e->stride[g] = total;
        total *= e->radix[g];
    }
    e->committed_stride = total;
    e->total = total * AIRSPY_STEERING_SLAVE_COUNT;
}

static size_t encode(const encoding* const e,
    const size_t* const free_counts,
    const size_t committed)
{
    size_t index = committed * e->committed_stride;
    for (size_t g = 0; g < AIRSPY_STEERING_SLAVE_COUNT; ++g) {
        index += free_counts[g] * e->stride[g];
    }
    return index;
}

static void decode(const encoding* const e,
    size_t index,
    size_t* const free_counts,
    size_t* const committed)
{
    *committed = index / e->committed_stride;
    index %= e->committed_stride;
    for (size_t g = 0; g < AIRSPY_STEERING_SLAVE_COUNT; ++g) {
        free_counts[g] = (index / e->stride[g]) % e->radix[g];
    }
}

static size_t occupied_slave_count(const size_t* const free_counts)
{
    size_t n = 0;
    for (size_t g = 0; g < AIRSPY_STEERING_SLAVE_COUNT; ++g) {
        if (free_counts[g] != 0) {
            ++n;
        }
    }
    return n;
}

/* A legal destination is on a different slave than the committed one. */
static int steering_possible(const size_t* const free_counts,
    const size_t committed)
{
    for (size_t g = 0; g < AIRSPY_STEERING_SLAVE_COUNT; ++g) {
        if (g != committed && free_counts[g] != 0) {
            return 1;
        }
    }
    return 0;
}

airspy_steering_result airspy_steering_check(
    const airspy_steering_layout* const layout,
    const int enforce_reserve,
    airspy_steering_report* const report)
{
    if (layout == NULL || report == NULL) {
        return AIRSPY_STEERING_BAD_ARGUMENT;
    }
    memset(report, 0, sizeof(*report));

    encoding e;
    encoding_init(&e, layout);

    unsigned char* const seen = calloc(e.total, 1);
    size_t* const queue = malloc(e.total * sizeof(size_t));
    if (seen == NULL || queue == NULL) {
        free(seen);
        free(queue);
        return AIRSPY_STEERING_NO_MEMORY;
    }

    size_t head = 0;
    size_t tail = 0;
    size_t free_counts[AIRSPY_STEERING_SLAVE_COUNT];
    size_t next_counts[AIRSPY_STEERING_SLAVE_COUNT];

    /* Start from a fully retired ring, whichever slave is committed first. */
    for (size_t g = 0; g < AIRSPY_STEERING_SLAVE_COUNT; ++g) {
        if (layout->slave_size[g] == 0) {
            continue;
        }
        const size_t start = encode(&e, layout->slave_size, g);
        if (!seen[start]) {
            seen[start] = 1;
            queue[tail++] = start;
        }
    }

    while (head != tail) {
        const size_t index = queue[head++];
        size_t committed;
        decode(&e, index, free_counts, &committed);
        ++report->states_explored;

        size_t free_total = 0;
        for (size_t g = 0; g < AIRSPY_STEERING_SLAVE_COUNT; ++g) {
            free_total += free_counts[g];
        }
        const size_t submitted = layout->bank_count - free_total;
        if (submitted > report->max_submitted) {
            report->max_submitted = submitted;
        }

        if (!steering_possible(free_counts, committed)) {
            if (report->stuck_states == 0) {
                memcpy(report->stuck_free, free_counts, sizeof(free_counts));
                report->stuck_committed_slave = committed;
            }
            ++report->stuck_states;
            continue;
        }

#define PUSH(counts, commit)                                                   \
    do {                                                                       \
        const size_t next_index = encode(&e, (counts), (commit));              \
        if (!seen[next_index]) {                                               \
            seen[next_index] = 1;                                              \
            queue[tail++] = next_index;                                        \
        }                                                                      \
    } while (0)

        /* Steer the bank after next onto a different slave. */
        for (size_t g = 0; g < AIRSPY_STEERING_SLAVE_COUNT; ++g) {
            if (g != committed && free_counts[g] != 0) {
                PUSH(free_counts, g);
            }
        }

        /*
         * M0 submits a completed bank. The bank being filled sits on the
         * committed slave and is not yet completed, so submitting from that
         * slave needs one more than elsewhere.
         */
        for (size_t g = 0; g < AIRSPY_STEERING_SLAVE_COUNT; ++g) {
            const size_t need = (g == committed) ? 2u : 1u;
            if (free_counts[g] < need) {
                continue;
            }
            memcpy(next_counts, free_counts, sizeof(free_counts));
            next_counts[g]--;
            if (enforce_reserve && occupied_slave_count(next_counts) < 2u) {
                continue; /* the submission rule refuses this */
            }
            PUSH(next_counts, committed);
        }

        /* USB retires a dTD and the bank returns. */
        for (size_t g = 0; g < AIRSPY_STEERING_SLAVE_COUNT; ++g) {
            if (free_counts[g] >= layout->slave_size[g]) {
                continue;
            }
            memcpy(next_counts, free_counts, sizeof(free_counts));
            next_counts[g]++;
            PUSH(next_counts, committed);
        }

#undef PUSH
    }

    free(seen);
    free(queue);
    return AIRSPY_STEERING_OK;
}
