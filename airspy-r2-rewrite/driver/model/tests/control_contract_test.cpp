#include "airspy_driver/legacy_control.hpp"

#include <array>
#include <cassert>
#include <cstdint>

int main()
{
    using namespace airspy::driver;

    static_assert(receiver_mode_control(ReceiverMode::off)
        == ControlSetup{
            0x40,
            LegacyRequest::receiver_mode,
            0,
            0,
            0,
            0});
    static_assert(receiver_mode_control(ReceiverMode::receive).value == 1);
    static_assert(receiver_mode_control(ReceiverMode::armed).value == 2);

    static_assert(samplerate_control(0)
        == ControlSetup{
            0xc0,
            LegacyRequest::set_samplerate,
            0,
            0,
            1,
            1});
    static_assert(samplerate_control(1).index == 1);

    static_assert(packing_control(false).index == 0);
    static_assert(packing_control(true).index == 1);

    constexpr ControlSetup lna =
        gain_control(LegacyRequest::set_lna_gain, 14);
    static_assert(lna.request_type == 0xc0);
    static_assert(lna.request == LegacyRequest::set_lna_gain);
    static_assert(lna.index == 14);
    static_assert(lna.length == 1);

    constexpr FrequencyControl frequency = frequency_control(0x12345678U);
    static_assert(frequency.setup.request_type == 0x40);
    static_assert(frequency.setup.request == LegacyRequest::set_frequency);
    static_assert(frequency.setup.length == 4);
    static_assert(frequency.setup.expected_length == 4);
    static_assert(
        frequency.payload
        == std::array<std::uint8_t, 4>{0x78, 0x56, 0x34, 0x12});

    return 0;
}
