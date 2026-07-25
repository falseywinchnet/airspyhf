#include "airspy_driver/legacy_unpack.hpp"

namespace airspy::driver {
namespace {

std::uint32_t read_u32_le(const std::uint8_t* source) noexcept
{
    return static_cast<std::uint32_t>(source[0])
        | (static_cast<std::uint32_t>(source[1]) << 8)
        | (static_cast<std::uint32_t>(source[2]) << 16)
        | (static_cast<std::uint32_t>(source[3]) << 24);
}

} // namespace

UnpackResult unpack_legacy_u12(
    const std::span<const std::uint8_t> input,
    const std::span<std::uint16_t> output) noexcept
{
    if (input.size() % 12 != 0) {
        return UnpackResult::input_size;
    }
    const std::size_t required_output = (input.size() / 12) * 8;
    if (output.size() < required_output) {
        return UnpackResult::output_size;
    }

    for (std::size_t source = 0, destination = 0;
         source < input.size();
         source += 12, destination += 8) {
        const std::uint32_t first = read_u32_le(&input[source]);
        const std::uint32_t second = read_u32_le(&input[source + 4]);
        const std::uint32_t third = read_u32_le(&input[source + 8]);

        output[destination] = static_cast<std::uint16_t>((first >> 20) & 0xfff);
        output[destination + 1] =
            static_cast<std::uint16_t>((first >> 8) & 0xfff);
        output[destination + 2] = static_cast<std::uint16_t>(
            ((first & 0xff) << 4) | ((second >> 28) & 0xf));
        output[destination + 3] =
            static_cast<std::uint16_t>((second >> 16) & 0xfff);
        output[destination + 4] =
            static_cast<std::uint16_t>((second >> 4) & 0xfff);
        output[destination + 5] = static_cast<std::uint16_t>(
            ((second & 0xf) << 8) | ((third >> 24) & 0xff));
        output[destination + 6] =
            static_cast<std::uint16_t>((third >> 12) & 0xfff);
        output[destination + 7] =
            static_cast<std::uint16_t>(third & 0xfff);
    }
    return UnpackResult::ok;
}

} // namespace airspy::driver
