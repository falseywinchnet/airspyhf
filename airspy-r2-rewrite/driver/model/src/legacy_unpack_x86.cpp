#include "legacy_unpack_x86.hpp"

#if (defined(__x86_64__) || defined(_M_X64)) \
    && (defined(__GNUC__) || defined(__clang__))

#include <immintrin.h>

namespace airspy::driver::detail {
namespace {

#define AIRSPY_TARGET_SSE41 \
    __attribute__((target("ssse3,sse4.1")))
#define AIRSPY_TARGET_AVX2 \
    __attribute__((target("avx2")))
#define AIRSPY_ALWAYS_INLINE __attribute__((always_inline)) inline

enum class X86UnpackIsa {
    scalar,
    sse41,
    avx2
};

X86UnpackIsa detect_unpack_isa() noexcept
{
    __builtin_cpu_init();
    if (__builtin_cpu_supports("avx2")) {
        return X86UnpackIsa::avx2;
    }
    if (__builtin_cpu_supports("ssse3")
        && __builtin_cpu_supports("sse4.1")) {
        return X86UnpackIsa::sse41;
    }
    return X86UnpackIsa::scalar;
}

X86UnpackIsa unpack_isa() noexcept
{
    static const X86UnpackIsa selected = detect_unpack_isa();
    return selected;
}

AIRSPY_TARGET_SSE41
AIRSPY_ALWAYS_INLINE __m128i unpack_indices_128() noexcept
{
    return _mm_setr_epi8(
        2, 3, 1, 2, 7, 0, 6, 7,
        4, 5, 11, 4, 9, 10, 8, 9);
}

AIRSPY_TARGET_SSE41
AIRSPY_ALWAYS_INLINE __m128i load_final_12(
    const std::uint8_t* const input) noexcept
{
    const __m128i low = _mm_loadl_epi64(
        reinterpret_cast<const __m128i*>(input));
    std::uint32_t high;
    __builtin_memcpy(&high, input + 8, sizeof(high));
    return _mm_insert_epi32(low, static_cast<int>(high), 2);
}

AIRSPY_TARGET_SSE41
AIRSPY_ALWAYS_INLINE __m128i decode_128(
    const __m128i bytes,
    const __m128i indices) noexcept
{
    const __m128i gathered = _mm_shuffle_epi8(bytes, indices);
    const __m128i shifted = _mm_srli_epi16(gathered, 4);
    return _mm_and_si128(
        _mm_blend_epi16(shifted, gathered, 0xaa),
        _mm_set1_epi16(0x0fff));
}

AIRSPY_TARGET_AVX2
AIRSPY_ALWAYS_INLINE __m256i decode_256(
    const __m256i bytes,
    const __m256i indices) noexcept
{
    const __m256i gathered = _mm256_shuffle_epi8(bytes, indices);
    const __m256i shifted = _mm256_srli_epi16(gathered, 4);
    return _mm256_and_si256(
        _mm256_blend_epi16(shifted, gathered, 0xaa),
        _mm256_set1_epi16(0x0fff));
}

AIRSPY_TARGET_SSE41
void unpack_sse_u16(
    const std::uint8_t* const input,
    const std::size_t groups,
    std::uint16_t* const output) noexcept
{
    const __m128i indices = unpack_indices_128();
    std::size_t group = 0;
    for (; group + 1U < groups; ++group) {
        const __m128i bytes = _mm_loadu_si128(
            reinterpret_cast<const __m128i*>(input + group * 12U));
        _mm_storeu_si128(
            reinterpret_cast<__m128i*>(output + group * 8U),
            decode_128(bytes, indices));
    }
    if (group < groups) {
        _mm_storeu_si128(
            reinterpret_cast<__m128i*>(output + group * 8U),
            decode_128(load_final_12(input + group * 12U), indices));
    }
}

AIRSPY_TARGET_SSE41
void unpack_sse_i16(
    const std::uint8_t* const input,
    const std::size_t groups,
    std::int16_t* const output) noexcept
{
    const __m128i indices = unpack_indices_128();
    const __m128i sign_flip = _mm_set1_epi16(
        -32768);
    std::size_t group = 0;
    for (; group + 1U < groups; ++group) {
        const __m128i bytes = _mm_loadu_si128(
            reinterpret_cast<const __m128i*>(input + group * 12U));
        const __m128i samples = decode_128(bytes, indices);
        _mm_storeu_si128(
            reinterpret_cast<__m128i*>(output + group * 8U),
            _mm_xor_si128(_mm_slli_epi16(samples, 4), sign_flip));
    }
    if (group < groups) {
        const __m128i samples =
            decode_128(load_final_12(input + group * 12U), indices);
        _mm_storeu_si128(
            reinterpret_cast<__m128i*>(output + group * 8U),
            _mm_xor_si128(_mm_slli_epi16(samples, 4), sign_flip));
    }
}

AIRSPY_TARGET_SSE41
void unpack_sse_f32(
    const std::uint8_t* const input,
    const std::size_t groups,
    float* const output) noexcept
{
    const __m128i indices = unpack_indices_128();
    const __m128i midpoint = _mm_set1_epi16(2048);
    const __m128 scale = _mm_set1_ps(1.0F / 2048.0F);
    std::size_t group = 0;
    for (; group < groups; ++group) {
        const __m128i bytes = group + 1U < groups
            ? _mm_loadu_si128(reinterpret_cast<const __m128i*>(
                  input + group * 12U))
            : load_final_12(input + group * 12U);
        const __m128i centered =
            _mm_sub_epi16(decode_128(bytes, indices), midpoint);
        const __m128i low = _mm_cvtepi16_epi32(centered);
        const __m128i high =
            _mm_cvtepi16_epi32(_mm_srli_si128(centered, 8));
        _mm_storeu_ps(
            output + group * 8U,
            _mm_mul_ps(_mm_cvtepi32_ps(low), scale));
        _mm_storeu_ps(
            output + group * 8U + 4U,
            _mm_mul_ps(_mm_cvtepi32_ps(high), scale));
    }
}

AIRSPY_TARGET_AVX2
void unpack_avx2_u16(
    const std::uint8_t* const input,
    const std::size_t groups,
    std::uint16_t* const output) noexcept
{
    const __m128i indices_128 = unpack_indices_128();
    const __m256i indices = _mm256_broadcastsi128_si256(indices_128);
    std::size_t group = 0;
    for (; group + 2U < groups; group += 2U) {
        const std::uint8_t* const source = input + group * 12U;
        __m256i bytes = _mm256_castsi128_si256(
            _mm_loadu_si128(reinterpret_cast<const __m128i*>(source)));
        bytes = _mm256_inserti128_si256(
            bytes,
            _mm_loadu_si128(
                reinterpret_cast<const __m128i*>(source + 12U)),
            1);
        _mm256_storeu_si256(
            reinterpret_cast<__m256i*>(output + group * 8U),
            decode_256(bytes, indices));
    }
    for (; group + 1U < groups; ++group) {
        const __m128i bytes = _mm_loadu_si128(
            reinterpret_cast<const __m128i*>(input + group * 12U));
        _mm_storeu_si128(
            reinterpret_cast<__m128i*>(output + group * 8U),
            decode_128(bytes, indices_128));
    }
    if (group < groups) {
        _mm_storeu_si128(
            reinterpret_cast<__m128i*>(output + group * 8U),
            decode_128(
                load_final_12(input + group * 12U),
                indices_128));
    }
}

AIRSPY_TARGET_AVX2
void unpack_avx2_i16(
    const std::uint8_t* const input,
    const std::size_t groups,
    std::int16_t* const output) noexcept
{
    const __m128i indices_128 = unpack_indices_128();
    const __m256i indices = _mm256_broadcastsi128_si256(indices_128);
    const __m256i sign_flip = _mm256_set1_epi16(
        -32768);
    std::size_t group = 0;
    for (; group + 2U < groups; group += 2U) {
        const std::uint8_t* const source = input + group * 12U;
        __m256i bytes = _mm256_castsi128_si256(
            _mm_loadu_si128(reinterpret_cast<const __m128i*>(source)));
        bytes = _mm256_inserti128_si256(
            bytes,
            _mm_loadu_si128(
                reinterpret_cast<const __m128i*>(source + 12U)),
            1);
        const __m256i samples = decode_256(bytes, indices);
        _mm256_storeu_si256(
            reinterpret_cast<__m256i*>(output + group * 8U),
            _mm256_xor_si256(
                _mm256_slli_epi16(samples, 4),
                sign_flip));
    }
    for (; group + 1U < groups; ++group) {
        const __m128i bytes = _mm_loadu_si128(
            reinterpret_cast<const __m128i*>(input + group * 12U));
        const __m128i samples = decode_128(bytes, indices_128);
        _mm_storeu_si128(
            reinterpret_cast<__m128i*>(output + group * 8U),
            _mm_xor_si128(
                _mm_slli_epi16(samples, 4),
                _mm256_castsi256_si128(sign_flip)));
    }
    if (group < groups) {
        const __m128i samples = decode_128(
            load_final_12(input + group * 12U), indices_128);
        _mm_storeu_si128(
            reinterpret_cast<__m128i*>(output + group * 8U),
            _mm_xor_si128(
                _mm_slli_epi16(samples, 4),
                _mm256_castsi256_si128(sign_flip)));
    }
}

AIRSPY_TARGET_AVX2
void unpack_avx2_f32(
    const std::uint8_t* const input,
    const std::size_t groups,
    float* const output) noexcept
{
    const __m128i indices_128 = unpack_indices_128();
    const __m256i indices = _mm256_broadcastsi128_si256(indices_128);
    const __m256i midpoint = _mm256_set1_epi16(2048);
    const __m256 scale = _mm256_set1_ps(1.0F / 2048.0F);
    std::size_t group = 0;
    for (; group + 2U < groups; group += 2U) {
        const std::uint8_t* const source = input + group * 12U;
        __m256i bytes = _mm256_castsi128_si256(
            _mm_loadu_si128(reinterpret_cast<const __m128i*>(source)));
        bytes = _mm256_inserti128_si256(
            bytes,
            _mm_loadu_si128(
                reinterpret_cast<const __m128i*>(source + 12U)),
            1);
        const __m256i centered =
            _mm256_sub_epi16(decode_256(bytes, indices), midpoint);
        const __m256 low = _mm256_cvtepi32_ps(
            _mm256_cvtepi16_epi32(
                _mm256_castsi256_si128(centered)));
        const __m256 high = _mm256_cvtepi32_ps(
            _mm256_cvtepi16_epi32(
                _mm256_extracti128_si256(centered, 1)));
        _mm256_storeu_ps(
            output + group * 8U,
            _mm256_mul_ps(low, scale));
        _mm256_storeu_ps(
            output + group * 8U + 8U,
            _mm256_mul_ps(high, scale));
    }
    for (; group < groups; ++group) {
        const __m128i bytes = group + 1U < groups
            ? _mm_loadu_si128(reinterpret_cast<const __m128i*>(
                  input + group * 12U))
            : load_final_12(input + group * 12U);
        const __m128i centered = _mm_sub_epi16(
            decode_128(bytes, indices_128),
            _mm256_castsi256_si128(midpoint));
        const __m128i low = _mm_cvtepi16_epi32(centered);
        const __m128i high =
            _mm_cvtepi16_epi32(_mm_srli_si128(centered, 8));
        const __m128 scale_128 = _mm256_castps256_ps128(scale);
        _mm_storeu_ps(
            output + group * 8U,
            _mm_mul_ps(_mm_cvtepi32_ps(low), scale_128));
        _mm_storeu_ps(
            output + group * 8U + 4U,
            _mm_mul_ps(_mm_cvtepi32_ps(high), scale_128));
    }
}

} // namespace

bool unpack_x86_u16(
    const std::uint8_t* const input,
    const std::size_t groups,
    std::uint16_t* const output) noexcept
{
    switch (unpack_isa()) {
    case X86UnpackIsa::avx2:
        unpack_avx2_u16(input, groups, output);
        return true;
    case X86UnpackIsa::sse41:
        unpack_sse_u16(input, groups, output);
        return true;
    case X86UnpackIsa::scalar:
        return false;
    }
    return false;
}

bool unpack_x86_i16(
    const std::uint8_t* const input,
    const std::size_t groups,
    std::int16_t* const output) noexcept
{
    switch (unpack_isa()) {
    case X86UnpackIsa::avx2:
        unpack_avx2_i16(input, groups, output);
        return true;
    case X86UnpackIsa::sse41:
        unpack_sse_i16(input, groups, output);
        return true;
    case X86UnpackIsa::scalar:
        return false;
    }
    return false;
}

bool unpack_x86_f32(
    const std::uint8_t* const input,
    const std::size_t groups,
    float* const output) noexcept
{
    switch (unpack_isa()) {
    case X86UnpackIsa::avx2:
        unpack_avx2_f32(input, groups, output);
        return true;
    case X86UnpackIsa::sse41:
        unpack_sse_f32(input, groups, output);
        return true;
    case X86UnpackIsa::scalar:
        return false;
    }
    return false;
}

} // namespace airspy::driver::detail

#endif
