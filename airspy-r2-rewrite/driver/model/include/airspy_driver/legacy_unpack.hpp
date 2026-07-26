#ifndef AIRSPY_DRIVER_LEGACY_UNPACK_HPP
#define AIRSPY_DRIVER_LEGACY_UNPACK_HPP

#include <cstddef>
#include <cstdint>
#include <span>

namespace airspy::driver {

enum class UnpackResult {
    ok,
    input_size,
    output_size
};

UnpackResult unpack_legacy_u12(
    std::span<const std::uint8_t> input,
    std::span<std::uint16_t> output) noexcept;

UnpackResult unpack_legacy_u12_to_i16(
    std::span<const std::uint8_t> input,
    std::span<std::int16_t> output) noexcept;

UnpackResult unpack_legacy_u12_to_f32(
    std::span<const std::uint8_t> input,
    std::span<float> output) noexcept;

} // namespace airspy::driver

#endif
