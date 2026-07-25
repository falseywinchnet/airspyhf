#include "airspy_firmware/steering_model.h"

#include <assert.h>
#include <stdio.h>

/*
 * The bank addresses programmed by the firmware.
 *
 * NOTE: these are duplicated from airspy_m4.c:182-191. The model does not share
 * headers with the device build, so there is currently no single definition to
 * read. Moving the table into a shared header and having both sides consume it
 * is the remaining step; until then a change on one side must be mirrored here,
 * and this test is what catches the consequences of getting it wrong.
 */
static const uint32_t bank_addresses[] = {
    0x20004000u,
    0x10008000u,
    0x10084000u,
    0x1000c000u,
    0x20008000u,
    0x10010000u,
    0x10088000u,
    0x10014000u,
    0x1008c000u,
    0x10018000u
};

#define BANK_COUNT (sizeof(bank_addresses) / sizeof(bank_addresses[0]))

int main(void)
{
    airspy_steering_layout layout;
    airspy_steering_report report;

    assert(airspy_steering_layout_init(&layout, bank_addresses, BANK_COUNT)
        == AIRSPY_STEERING_OK);
    assert(layout.bank_count == BANK_COUNT);

    /* Every bank must land on a slave port we know about. */
    for (size_t i = 0; i < BANK_COUNT; ++i) {
        assert(layout.slave[i] != AIRSPY_STEERING_SLAVE_NONE);
    }

    /* The banks span more than two slave ports, which is what gives steering
       room to alternate without being forced back onto one port. */
    size_t used = 0;
    for (size_t g = 0; g < AIRSPY_STEERING_SLAVE_COUNT; ++g) {
        if (layout.slave_size[g] != 0) {
            ++used;
        }
    }
    assert(used >= 2);

    /* The order actually programmed alternates, including the wrap. This is
       what V5c depends on and it must keep holding. */
    assert(airspy_steering_static_order_ok(&layout));

    /* With the submission rule, steering always has somewhere to go. */
    assert(airspy_steering_check(&layout, 1, &report) == AIRSPY_STEERING_OK);
    printf("with reserve rule:    %zu states, %zu stuck, max submitted %zu\n",
        report.states_explored, report.stuck_states, report.max_submitted);
    assert(report.stuck_states == 0);
    assert(report.states_explored > 0);

    /* Depth is bank_count - 2: one bank is being filled and one must stay
       available for the steering decision. */
    assert(report.max_submitted == BANK_COUNT - 2u);

    /*
     * Without the rule, states exist with no legal destination. This is not a
     * shortage of free banks - the example below has several - but a case where
     * every free bank sits on the slave that is already committed. A rule
     * phrased as a free-bank count cannot express that, which is why the real
     * rule talks about slave ports.
     */
    assert(airspy_steering_check(&layout, 0, &report) == AIRSPY_STEERING_OK);
    printf("without reserve rule: %zu states, %zu stuck, max submitted %zu\n",
        report.states_explored, report.stuck_states, report.max_submitted);
    assert(report.stuck_states > 0);

    size_t stuck_free_total = 0;
    for (size_t g = 0; g < AIRSPY_STEERING_SLAVE_COUNT; ++g) {
        stuck_free_total += report.stuck_free[g];
    }
    printf("  example stuck state: %zu banks free, all on slave %zu, "
           "next committed to slave %zu\n",
        stuck_free_total,
        report.stuck_committed_slave,
        report.stuck_committed_slave);
    assert(stuck_free_total > 0);

    /* A layout entirely on one slave port cannot alternate at all. */
    static const uint32_t one_slave[] = {
        0x10008000u, 0x1000c000u, 0x10010000u
    };
    airspy_steering_layout degenerate;
    assert(airspy_steering_layout_init(&degenerate, one_slave, 3)
        == AIRSPY_STEERING_OK);
    assert(!airspy_steering_static_order_ok(&degenerate));
    assert(airspy_steering_check(&degenerate, 1, &report)
        == AIRSPY_STEERING_OK);
    assert(report.stuck_states > 0);

    /* An address outside the mapped SRAM regions is rejected rather than
       silently grouped. */
    static const uint32_t unmapped[] = {0x18000000u, 0x10008000u};
    airspy_steering_layout bad;
    assert(airspy_steering_layout_init(&bad, unmapped, 2)
        == AIRSPY_STEERING_UNMAPPED_ADDRESS);

    return 0;
}
