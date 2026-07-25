#ifndef AIRSPY_FIRMWARE_STEERING_MODEL_H
#define AIRSPY_FIRMWARE_STEERING_MODEL_H

/*
 * Steering safety for the capture ring.
 *
 * GPDMA writes banks back to back. Two consecutive destinations must not lie on
 * the same AHB slave port: V5/V5b put the last two 16 KiB destinations
 * consecutively in local SRAM1 and produced one repeatable ADC FIFO overflow per
 * ten-bank revolution, which alternating removed. UM10503 3.6 gives the reason -
 * masters sharing one slave port arbitrate round robin, and GPDMA bursts up to
 * eight beats.
 *
 * V5c satisfies that by fixing the bank order at link time. If the destination
 * is instead chosen at run time, a link-time assertion proves nothing, so the
 * property has to be established over the reachable states instead.
 *
 * Steering only ever consults group level information, so the per-group counts
 * of not-yet-submitted banks are a sound abstraction of the concrete bank
 * states. That space is a few hundred states rather than tens of millions, which
 * makes this an ordinary test rather than a model checking exercise.
 */

#include <stddef.h>
#include <stdint.h>

enum {
    /* AHB slave ports that can hold capture banks (UM10503 memory map). */
    AIRSPY_STEERING_SLAVE_COUNT = 4,
    AIRSPY_STEERING_MAX_BANKS = 16,
    AIRSPY_STEERING_SLAVE_NONE = 0xFFu
};

typedef enum {
    AIRSPY_STEERING_OK = 0,
    AIRSPY_STEERING_BAD_ARGUMENT,
    AIRSPY_STEERING_UNMAPPED_ADDRESS,
    AIRSPY_STEERING_TOO_MANY_BANKS,
    AIRSPY_STEERING_NO_MEMORY
} airspy_steering_result;

typedef struct {
    uint32_t addresses[AIRSPY_STEERING_MAX_BANKS];
    uint8_t slave[AIRSPY_STEERING_MAX_BANKS];
    size_t bank_count;
    size_t slave_size[AIRSPY_STEERING_SLAVE_COUNT];
} airspy_steering_layout;

typedef struct {
    size_t states_explored;
    size_t stuck_states;
    /* Deepest submission reachable: banks simultaneously owned by USB. */
    size_t max_submitted;
    /* First stuck state found, for diagnosis. Valid when stuck_states != 0. */
    size_t stuck_free[AIRSPY_STEERING_SLAVE_COUNT];
    size_t stuck_committed_slave;
} airspy_steering_report;

/* Which AHB slave port an address lives on, or AIRSPY_STEERING_SLAVE_NONE. */
uint8_t airspy_steering_slave_of(uint32_t address);

airspy_steering_result airspy_steering_layout_init(
    airspy_steering_layout* layout,
    const uint32_t* addresses,
    size_t count);

/*
 * The fixed order actually programmed: every consecutive pair, including the
 * wrap from last back to first, must differ in slave port. This is what V5c
 * relies on today.
 */
int airspy_steering_static_order_ok(const airspy_steering_layout* layout);

/*
 * Enumerate every reachable abstract state and report whether steering can ever
 * be left with no legal destination.
 *
 * enforce_reserve applies the submission rule: never submit a bank if that would
 * leave the not-yet-submitted banks sitting on fewer than two distinct slave
 * ports. Passing 0 shows what happens without it.
 */
airspy_steering_result airspy_steering_check(
    const airspy_steering_layout* layout,
    int enforce_reserve,
    airspy_steering_report* report);

#endif /* AIRSPY_FIRMWARE_STEERING_MODEL_H */
