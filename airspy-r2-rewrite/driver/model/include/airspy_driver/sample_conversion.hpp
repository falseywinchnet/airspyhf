#ifndef AIRSPY_DRIVER_SAMPLE_CONVERSION_HPP
#define AIRSPY_DRIVER_SAMPLE_CONVERSION_HPP

#include <cstdint>
#include <span>

namespace airspy::driver {

enum class ConversionResult : std::uint8_t {
    ok,
    output_size
};

ConversionResult convert_u12_to_i16(
    std::span<const std::uint16_t> input,
    std::span<std::int16_t> output) noexcept;

ConversionResult convert_u12_to_f32(
    std::span<const std::uint16_t> input,
    std::span<float> output) noexcept;

} // namespace airspy::driver

#endif
