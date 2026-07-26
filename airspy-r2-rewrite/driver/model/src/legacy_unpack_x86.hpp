#ifndef AIRSPY_DRIVER_LEGACY_UNPACK_X86_HPP
#define AIRSPY_DRIVER_LEGACY_UNPACK_X86_HPP

#include <cstddef>
#include <cstdint>

namespace airspy::driver::detail {

#if (defined(__x86_64__) || defined(_M_X64)) && \
    (defined(__GNUC__) || defined(__clang__))

#define AIRSPY_DRIVER_X86_SIMD 1

bool unpack_x86_u16(
    const std::uint8_t* input,
    std::size_t groups,
    std::uint16_t* output) noexcept;

bool unpack_x86_i16(
    const std::uint8_t* input,
    std::size_t groups,
    std::int16_t* output) noexcept;

bool unpack_x86_f32(
    const std::uint8_t* input,
    std::size_t groups,
    float* output) noexcept;

#endif

} // namespace airspy::driver::detail

#endif
