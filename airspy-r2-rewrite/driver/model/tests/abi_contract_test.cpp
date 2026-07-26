#include "airspy.h"

#include <cstddef>
#include <cstdint>
#include <type_traits>

static_assert(AIRSPY_VER_MAJOR == 1);
static_assert(AIRSPY_VER_MINOR == 0);
static_assert(AIRSPY_VER_REVISION == 12);

static_assert(AIRSPY_SUCCESS == 0);
static_assert(AIRSPY_TRUE == 1);
static_assert(AIRSPY_ERROR_INVALID_PARAM == -2);
static_assert(AIRSPY_ERROR_LIBUSB == -1000);
static_assert(AIRSPY_ERROR_THREAD == -1001);
static_assert(AIRSPY_ERROR_OTHER == -9999);

static_assert(AIRSPY_SAMPLE_FLOAT32_IQ == 0);
static_assert(AIRSPY_SAMPLE_FLOAT32_REAL == 1);
static_assert(AIRSPY_SAMPLE_INT16_IQ == 2);
static_assert(AIRSPY_SAMPLE_INT16_REAL == 3);
static_assert(AIRSPY_SAMPLE_UINT16_REAL == 4);
static_assert(AIRSPY_SAMPLE_RAW == 5);
static_assert(AIRSPY_SAMPLE_END == 6);

static_assert(RECEIVER_MODE_OFF == 0);
static_assert(RECEIVER_MODE_RX == 1);
static_assert(RECEIVER_MODE_ARMED == 2);

static_assert(AIRSPY_RECEIVER_MODE == 1);
static_assert(AIRSPY_SET_SAMPLERATE == 12);
static_assert(AIRSPY_SET_FREQ == 13);
static_assert(AIRSPY_GET_SAMPLERATES == 25);
static_assert(AIRSPY_SET_PACKING == 26);

static_assert(std::is_standard_layout_v<airspy_transfer_t>);
static_assert(std::is_trivially_copyable_v<airspy_transfer_t>);
static_assert(offsetof(airspy_transfer_t, device) == 0);
static_assert(offsetof(airspy_transfer_t, ctx) > offsetof(airspy_transfer_t, device));
static_assert(offsetof(airspy_transfer_t, samples) > offsetof(airspy_transfer_t, ctx));
static_assert(
    offsetof(airspy_transfer_t, sample_count)
    > offsetof(airspy_transfer_t, samples));
static_assert(
    offsetof(airspy_transfer_t, dropped_samples)
    > offsetof(airspy_transfer_t, sample_count));
static_assert(
    offsetof(airspy_transfer_t, sample_type)
    > offsetof(airspy_transfer_t, dropped_samples));

int main()
{
    return 0;
}
