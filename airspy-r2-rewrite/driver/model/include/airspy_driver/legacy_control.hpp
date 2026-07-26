#ifndef AIRSPY_DRIVER_LEGACY_CONTROL_HPP
#define AIRSPY_DRIVER_LEGACY_CONTROL_HPP

#include "airspy_driver/prepared_start.hpp"

#include <array>
#include <cstdint>

namespace airspy::driver {

inline constexpr std::uint8_t vendor_device_out = 0x40;
inline constexpr std::uint8_t vendor_device_in = 0xc0;

enum class LegacyRequest : std::uint8_t {
    receiver_mode = 1,
    set_samplerate = 12,
    set_frequency = 13,
    set_lna_gain = 14,
    set_mixer_gain = 15,
    set_vga_gain = 16,
    set_lna_agc = 17,
    set_mixer_agc = 18,
    get_samplerates = 25,
    set_packing = 26
};

struct ControlSetup {
    std::uint8_t request_type{0};
    LegacyRequest request{LegacyRequest::receiver_mode};
    std::uint16_t value{0};
    std::uint16_t index{0};
    std::uint16_t length{0};
    std::uint16_t expected_length{0};

    auto operator<=>(const ControlSetup&) const = default;
};

struct FrequencyControl {
    ControlSetup setup{};
    std::array<std::uint8_t, 4> payload{};
};

[[nodiscard]] constexpr ControlSetup receiver_mode_control(
    ReceiverMode mode) noexcept
{
    return {
        vendor_device_out,
        LegacyRequest::receiver_mode,
        static_cast<std::uint16_t>(mode),
        0,
        0,
        0};
}

[[nodiscard]] constexpr ControlSetup samplerate_control(
    std::uint16_t firmware_rate) noexcept
{
    return {
        vendor_device_in,
        LegacyRequest::set_samplerate,
        0,
        firmware_rate,
        1,
        1};
}

[[nodiscard]] constexpr ControlSetup packing_control(
    const bool enabled) noexcept
{
    return {
        vendor_device_in,
        LegacyRequest::set_packing,
        0,
        static_cast<std::uint16_t>(enabled ? 1 : 0),
        1,
        1};
}

[[nodiscard]] constexpr ControlSetup gain_control(
    LegacyRequest request,
    std::uint8_t value) noexcept
{
    return {
        vendor_device_in,
        request,
        0,
        value,
        1,
        1};
}

[[nodiscard]] constexpr FrequencyControl frequency_control(
    const std::uint32_t frequency_hz) noexcept
{
    return {
        {
            vendor_device_out,
            LegacyRequest::set_frequency,
            0,
            0,
            4,
            4},
        {
            static_cast<std::uint8_t>(frequency_hz),
            static_cast<std::uint8_t>(frequency_hz >> 8),
            static_cast<std::uint8_t>(frequency_hz >> 16),
            static_cast<std::uint8_t>(frequency_hz >> 24)}};
}

} // namespace airspy::driver

#endif
