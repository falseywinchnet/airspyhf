#include <arm_neon.h>

#include <cstddef>
#include <cstdint>
#include <span>

namespace {

alignas(16) constexpr std::uint8_t indices_data[16] = {
    2, 3, 1, 2, 7, 0, 6, 7, 4, 5, 11, 4, 9, 10, 8, 9
};
alignas(16) constexpr std::int16_t shifts_data[8] = {
    -4, 0, -4, 0, -4, 0, -4, 0
};

uint16x8_t unpack_vector(
    const std::uint8_t* const input,
    const uint8x16_t indices,
    const int16x8_t shifts) noexcept
{
    const uint8x16_t bytes = vld1q_u8(input);
    const uint8x16_t gathered = vqtbl1q_u8(bytes, indices);
    const uint16x8_t candidates = vreinterpretq_u16_u8(gathered);
    return vandq_u16(
        vshlq_u16(candidates, shifts),
        vdupq_n_u16(0x0fffU));
}

void unpack_one_scalar(
    const std::uint8_t* const input,
    std::uint16_t* const output) noexcept
{
    const auto word = [input](const std::size_t offset) {
        return static_cast<std::uint32_t>(input[offset])
            | (static_cast<std::uint32_t>(input[offset + 1]) << 8)
            | (static_cast<std::uint32_t>(input[offset + 2]) << 16)
            | (static_cast<std::uint32_t>(input[offset + 3]) << 24);
    };
    const std::uint32_t first = word(0);
    const std::uint32_t second = word(4);
    const std::uint32_t third = word(8);

    output[0] = static_cast<std::uint16_t>(first >> 20);
    output[1] = static_cast<std::uint16_t>((first >> 8) & 0xfffU);
    output[2] = static_cast<std::uint16_t>(
        ((first & 0xffU) << 4) | (second >> 28));
    output[3] = static_cast<std::uint16_t>((second >> 16) & 0xfffU);
    output[4] = static_cast<std::uint16_t>((second >> 4) & 0xfffU);
    output[5] = static_cast<std::uint16_t>(
        ((second & 0xfU) << 8) | (third >> 24));
    output[6] = static_cast<std::uint16_t>((third >> 12) & 0xfffU);
    output[7] = static_cast<std::uint16_t>(third & 0xfffU);
}

} // namespace

bool unpack_neon(
    const std::span<const std::uint8_t> input,
    const std::span<std::uint16_t> output) noexcept
{
    if (input.empty() || input.size() % 12U != 0U
        || output.size() < (input.size() / 12U) * 8U) {
        return false;
    }

    // Each pair selects the low and high byte of one 16-bit candidate.
    // Even candidates are shifted right by four bits; odd candidates already
    // have the packed sample in their low twelve bits.
    const std::size_t groups = input.size() / 12U;
    const uint8x16_t indices = vld1q_u8(indices_data);
    const int16x8_t shifts = vld1q_s16(shifts_data);
    std::size_t group = 0;
    for (; group + 1U < groups; ++group) {
        vst1q_u16(
            output.data() + group * 8U,
            unpack_vector(
                input.data() + group * 12U, indices, shifts));
    }

    unpack_one_scalar(
        input.data() + group * 12U,
        output.data() + group * 8U);
    return true;
}

bool unpack_neon_i16(
    const std::span<const std::uint8_t> input,
    const std::span<std::int16_t> output) noexcept
{
    if (input.empty() || input.size() % 12U != 0U
        || output.size() < (input.size() / 12U) * 8U) {
        return false;
    }

    const std::size_t groups = input.size() / 12U;
    const uint8x16_t indices = vld1q_u8(indices_data);
    const int16x8_t shifts = vld1q_s16(shifts_data);
    std::size_t group = 0;
    for (; group + 1U < groups; ++group) {
        const int16x8_t centered = vsubq_s16(
            vreinterpretq_s16_u16(
                unpack_vector(
                    input.data() + group * 12U, indices, shifts)),
            vdupq_n_s16(2048));
        vst1q_s16(
            output.data() + group * 8U,
            vshlq_n_s16(centered, 4));
    }

    std::uint16_t tail[8];
    unpack_one_scalar(input.data() + group * 12U, tail);
    for (std::size_t index = 0; index < 8U; ++index) {
        output[group * 8U + index] = static_cast<std::int16_t>(
            (static_cast<int>(tail[index]) - 2048) * 16);
    }
    return true;
}

bool unpack_neon_f32(
    const std::span<const std::uint8_t> input,
    const std::span<float> output) noexcept
{
    if (input.empty() || input.size() % 12U != 0U
        || output.size() < (input.size() / 12U) * 8U) {
        return false;
    }

    const std::size_t groups = input.size() / 12U;
    const uint8x16_t indices = vld1q_u8(indices_data);
    const int16x8_t shifts = vld1q_s16(shifts_data);
    std::size_t group = 0;
    for (; group + 1U < groups; ++group) {
        const int16x8_t centered = vsubq_s16(
            vreinterpretq_s16_u16(
                unpack_vector(
                    input.data() + group * 12U, indices, shifts)),
            vdupq_n_s16(2048));
        const int32x4_t low = vmovl_s16(vget_low_s16(centered));
        const int32x4_t high = vmovl_high_s16(centered);
        vst1q_f32(
            output.data() + group * 8U,
            vcvtq_n_f32_s32(low, 11));
        vst1q_f32(
            output.data() + group * 8U + 4U,
            vcvtq_n_f32_s32(high, 11));
    }

    std::uint16_t tail[8];
    unpack_one_scalar(input.data() + group * 12U, tail);
    for (std::size_t index = 0; index < 8U; ++index) {
        output[group * 8U + index] =
            static_cast<float>(static_cast<int>(tail[index]) - 2048)
            * (1.0F / 2048.0F);
    }
    return true;
}
