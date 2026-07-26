#include "airspy_driver/sample_conversion.hpp"

namespace airspy::driver {
namespace {

constexpr int sample_midpoint = 2048;
constexpr int sample_shift_scale = 16;
constexpr float sample_float_scale = 1.0F / 2048.0F;

} // namespace

ConversionResult convert_u12_to_i16(
    const std::span<const std::uint16_t> input,
    const std::span<std::int16_t> output) noexcept
{
    if (output.size() < input.size()) {
        return ConversionResult::output_size;
    }
    for (std::size_t index = 0; index < input.size(); ++index) {
        const int centered = static_cast<int>(input[index] & 0x0fffU)
            - sample_midpoint;
        output[index] = static_cast<std::int16_t>(
            centered * sample_shift_scale);
    }
    return ConversionResult::ok;
}

ConversionResult convert_u12_to_f32(
    const std::span<const std::uint16_t> input,
    const std::span<float> output) noexcept
{
    if (output.size() < input.size()) {
        return ConversionResult::output_size;
    }
    for (std::size_t index = 0; index < input.size(); ++index) {
        const int centered = static_cast<int>(input[index] & 0x0fffU)
            - sample_midpoint;
        output[index] = static_cast<float>(centered) * sample_float_scale;
    }
    return ConversionResult::ok;
}

} // namespace airspy::driver
