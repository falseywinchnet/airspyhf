#include "airspy_driver/legacy_unpack.hpp"
#include "legacy_unpack_x86.hpp"

#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace airspy::driver {
namespace {

constexpr std::size_t packed_group_bytes = 12;
constexpr std::size_t unpacked_group_samples = 8;

#if defined(__GNUC__) && !defined(__clang__)
#define AIRSPY_NO_AUTOVECTOR \
    __attribute__((optimize("no-tree-vectorize")))
#else
#define AIRSPY_NO_AUTOVECTOR
#endif

std::uint32_t read_u32_le(const std::uint8_t* source) noexcept
{
    return static_cast<std::uint32_t>(source[0])
        | (static_cast<std::uint32_t>(source[1]) << 8)
        | (static_cast<std::uint32_t>(source[2]) << 16)
        | (static_cast<std::uint32_t>(source[3]) << 24);
}

void unpack_scalar_group(
    const std::uint8_t* const input,
    std::uint16_t* const output) noexcept
{
    const std::uint32_t first = read_u32_le(input);
    const std::uint32_t second = read_u32_le(input + 4);
    const std::uint32_t third = read_u32_le(input + 8);

    output[0] = static_cast<std::uint16_t>((first >> 20) & 0xfffU);
    output[1] = static_cast<std::uint16_t>((first >> 8) & 0xfffU);
    output[2] = static_cast<std::uint16_t>(
        ((first & 0xffU) << 4) | ((second >> 28) & 0xfU));
    output[3] = static_cast<std::uint16_t>((second >> 16) & 0xfffU);
    output[4] = static_cast<std::uint16_t>((second >> 4) & 0xfffU);
    output[5] = static_cast<std::uint16_t>(
        ((second & 0xfU) << 8) | ((third >> 24) & 0xffU));
    output[6] = static_cast<std::uint16_t>((third >> 12) & 0xfffU);
    output[7] = static_cast<std::uint16_t>(third & 0xfffU);
}

UnpackResult validate_spans(
    const std::size_t input_size,
    const std::size_t output_size) noexcept
{
    if (input_size % packed_group_bytes != 0) {
        return UnpackResult::input_size;
    }
    const std::size_t required_output =
        (input_size / packed_group_bytes) * unpacked_group_samples;
    return output_size < required_output
        ? UnpackResult::output_size
        : UnpackResult::ok;
}

AIRSPY_NO_AUTOVECTOR
void unpack_scalar_u16_groups(
    const std::span<const std::uint8_t> input,
    const std::span<std::uint16_t> output) noexcept
{
    const std::size_t groups = input.size() / packed_group_bytes;
    for (std::size_t group = 0; group < groups; ++group) {
        unpack_scalar_group(
            input.data() + group * packed_group_bytes,
            output.data() + group * unpacked_group_samples);
    }
}

AIRSPY_NO_AUTOVECTOR
void unpack_scalar_i16_groups(
    const std::span<const std::uint8_t> input,
    const std::span<std::int16_t> output) noexcept
{
    const std::size_t groups = input.size() / packed_group_bytes;
    for (std::size_t group = 0; group < groups; ++group) {
        std::uint16_t samples[unpacked_group_samples];
        unpack_scalar_group(
            input.data() + group * packed_group_bytes,
            samples);
        for (std::size_t index = 0;
             index < unpacked_group_samples;
             ++index) {
            output[group * unpacked_group_samples + index] =
                static_cast<std::int16_t>(
                    (static_cast<int>(samples[index]) - 2048) * 16);
        }
    }
}

AIRSPY_NO_AUTOVECTOR
void unpack_scalar_f32_groups(
    const std::span<const std::uint8_t> input,
    const std::span<float> output) noexcept
{
    const std::size_t groups = input.size() / packed_group_bytes;
    for (std::size_t group = 0; group < groups; ++group) {
        std::uint16_t samples[unpacked_group_samples];
        unpack_scalar_group(
            input.data() + group * packed_group_bytes,
            samples);
        for (std::size_t index = 0;
             index < unpacked_group_samples;
             ++index) {
            output[group * unpacked_group_samples + index] =
                static_cast<float>(
                    static_cast<int>(samples[index]) - 2048)
                * (1.0F / 2048.0F);
        }
    }
}

#if defined(__aarch64__)

alignas(16) constexpr std::uint8_t neon_indices[16] = {
    2, 3, 1, 2, 7, 0, 6, 7, 4, 5, 11, 4, 9, 10, 8, 9
};
alignas(16) constexpr std::int16_t neon_shifts[8] = {
    -4, 0, -4, 0, -4, 0, -4, 0
};

uint16x8_t unpack_neon_group(
    const std::uint8_t* const input,
    const uint8x16_t indices,
    const int16x8_t shifts) noexcept
{
    const uint8x16_t bytes = vld1q_u8(input);
    const uint8x16_t gathered = vqtbl1q_u8(bytes, indices);
    return vandq_u16(
        vshlq_u16(vreinterpretq_u16_u8(gathered), shifts),
        vdupq_n_u16(0x0fffU));
}

template<typename VectorFunction, typename ScalarFunction>
void transform_neon_groups(
    const std::span<const std::uint8_t> input,
    VectorFunction&& vector_function,
    ScalarFunction&& scalar_function)
{
    const uint8x16_t indices = vld1q_u8(neon_indices);
    const int16x8_t shifts = vld1q_s16(neon_shifts);
    const std::size_t groups = input.size() / packed_group_bytes;

    std::size_t group = 0;
    for (; group + 1U < groups; ++group) {
        vector_function(
            group,
            unpack_neon_group(
                input.data() + group * packed_group_bytes,
                indices,
                shifts));
    }
    if (group < groups) {
        std::uint16_t tail[unpacked_group_samples];
        unpack_scalar_group(
            input.data() + group * packed_group_bytes,
            tail);
        scalar_function(group, tail);
    }
}

#endif

} // namespace

UnpackResult unpack_legacy_u12(
    const std::span<const std::uint8_t> input,
    const std::span<std::uint16_t> output) noexcept
{
    const UnpackResult validation =
        validate_spans(input.size(), output.size());
    if (validation != UnpackResult::ok) {
        return validation;
    }

#if defined(__aarch64__)
    transform_neon_groups(
        input,
        [&output](const std::size_t group, const uint16x8_t samples) {
            vst1q_u16(
                output.data() + group * unpacked_group_samples,
                samples);
        },
        [&output](const std::size_t group, const std::uint16_t* tail) {
            for (std::size_t index = 0;
                 index < unpacked_group_samples;
                 ++index) {
                output[group * unpacked_group_samples + index] =
                    tail[index];
            }
        });
#elif defined(AIRSPY_DRIVER_X86_SIMD)
    if (!detail::unpack_x86_u16(
            input.data(),
            input.size() / packed_group_bytes,
            output.data())) {
        unpack_scalar_u16_groups(input, output);
    }
#else
    unpack_scalar_u16_groups(input, output);
#endif
    return UnpackResult::ok;
}

UnpackResult unpack_legacy_u12_to_i16(
    const std::span<const std::uint8_t> input,
    const std::span<std::int16_t> output) noexcept
{
    const UnpackResult validation =
        validate_spans(input.size(), output.size());
    if (validation != UnpackResult::ok) {
        return validation;
    }

#if defined(__aarch64__)
    transform_neon_groups(
        input,
        [&output](const std::size_t group, const uint16x8_t samples) {
            const int16x8_t centered = vsubq_s16(
                vreinterpretq_s16_u16(samples),
                vdupq_n_s16(2048));
            vst1q_s16(
                output.data() + group * unpacked_group_samples,
                vshlq_n_s16(centered, 4));
        },
        [&output](const std::size_t group, const std::uint16_t* tail) {
            for (std::size_t index = 0;
                 index < unpacked_group_samples;
                 ++index) {
                output[group * unpacked_group_samples + index] =
                    static_cast<std::int16_t>(
                        (static_cast<int>(tail[index]) - 2048) * 16);
            }
        });
#elif defined(AIRSPY_DRIVER_X86_SIMD)
    if (!detail::unpack_x86_i16(
            input.data(),
            input.size() / packed_group_bytes,
            output.data())) {
        unpack_scalar_i16_groups(input, output);
    }
#else
    unpack_scalar_i16_groups(input, output);
#endif
    return UnpackResult::ok;
}

UnpackResult unpack_legacy_u12_to_f32(
    const std::span<const std::uint8_t> input,
    const std::span<float> output) noexcept
{
    const UnpackResult validation =
        validate_spans(input.size(), output.size());
    if (validation != UnpackResult::ok) {
        return validation;
    }

#if defined(__aarch64__)
    transform_neon_groups(
        input,
        [&output](const std::size_t group, const uint16x8_t samples) {
            const int16x8_t centered = vsubq_s16(
                vreinterpretq_s16_u16(samples),
                vdupq_n_s16(2048));
            const int32x4_t low = vmovl_s16(vget_low_s16(centered));
            const int32x4_t high = vmovl_high_s16(centered);
            vst1q_f32(
                output.data() + group * unpacked_group_samples,
                vcvtq_n_f32_s32(low, 11));
            vst1q_f32(
                output.data() + group * unpacked_group_samples + 4U,
                vcvtq_n_f32_s32(high, 11));
        },
        [&output](const std::size_t group, const std::uint16_t* tail) {
            for (std::size_t index = 0;
                 index < unpacked_group_samples;
                 ++index) {
                output[group * unpacked_group_samples + index] =
                    static_cast<float>(
                        static_cast<int>(tail[index]) - 2048)
                    * (1.0F / 2048.0F);
            }
        });
#elif defined(AIRSPY_DRIVER_X86_SIMD)
    if (!detail::unpack_x86_f32(
            input.data(),
            input.size() / packed_group_bytes,
            output.data())) {
        unpack_scalar_f32_groups(input, output);
    }
#else
    unpack_scalar_f32_groups(input, output);
#endif
    return UnpackResult::ok;
}

} // namespace airspy::driver
