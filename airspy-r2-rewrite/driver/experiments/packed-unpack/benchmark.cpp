#include "airspy_driver/legacy_unpack.hpp"
#include "airspy_driver/sample_conversion.hpp"

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <vector>

#if defined(__aarch64__)
bool unpack_neon(
    std::span<const std::uint8_t> input,
    std::span<std::uint16_t> output) noexcept;
bool unpack_neon_i16(
    std::span<const std::uint8_t> input,
    std::span<std::int16_t> output) noexcept;
bool unpack_neon_f32(
    std::span<const std::uint8_t> input,
    std::span<float> output) noexcept;
#endif

namespace {

constexpr std::size_t input_bytes = 147456;
constexpr std::size_t output_samples = (input_bytes / 12) * 8;
constexpr std::size_t buffer_sets = 8;
constexpr std::size_t iterations = 8192;

template<typename Function>
double time_seconds(Function&& function)
{
    const auto begin = std::chrono::steady_clock::now();
    function();
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double>(end - begin).count();
}

} // namespace

int main()
{
    std::array<std::vector<std::uint8_t>, buffer_sets> packed;
    std::array<std::vector<std::uint16_t>, buffer_sets> unpacked;
    std::array<std::vector<std::int16_t>, buffer_sets> converted_i16;
    std::array<std::vector<float>, buffer_sets> converted_f32;

    std::uint32_t generator = 0x13579bdfU;
    for (std::size_t set = 0; set < buffer_sets; ++set) {
        packed[set].resize(input_bytes);
        unpacked[set].resize(output_samples);
        converted_i16[set].resize(output_samples);
        converted_f32[set].resize(output_samples);
        for (auto& byte : packed[set]) {
            generator = generator * 1664525U + 1013904223U;
            byte = static_cast<std::uint8_t>(generator >> 24);
        }
    }

    const double unpack_seconds = time_seconds([&] {
        for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
            const std::size_t set = iteration % buffer_sets;
            if (airspy::driver::unpack_legacy_u12(packed[set], unpacked[set])
                != airspy::driver::UnpackResult::ok) {
                std::abort();
            }
        }
    });

#if defined(__aarch64__)
    for (std::size_t set = 0; set < buffer_sets; ++set) {
        std::vector<std::uint16_t> reference(output_samples);
        if (airspy::driver::unpack_legacy_u12(packed[set], reference)
                != airspy::driver::UnpackResult::ok
            || !unpack_neon(packed[set], unpacked[set])
            || reference != unpacked[set]) {
            std::fputs("NEON correctness check failed\n", stderr);
            return 1;
        }
    }

    const double neon_seconds = time_seconds([&] {
        for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
            const std::size_t set = iteration % buffer_sets;
            if (!unpack_neon(packed[set], unpacked[set])) {
                std::abort();
            }
        }
    });

    for (std::size_t set = 0; set < buffer_sets; ++set) {
        std::vector<std::int16_t> reference_i16(output_samples);
        std::vector<float> reference_f32(output_samples);
        if (airspy::driver::convert_u12_to_i16(
                unpacked[set], reference_i16)
                != airspy::driver::ConversionResult::ok
            || airspy::driver::convert_u12_to_f32(
                   unpacked[set], reference_f32)
                != airspy::driver::ConversionResult::ok
            || !unpack_neon_i16(packed[set], converted_i16[set])
            || !unpack_neon_f32(packed[set], converted_f32[set])
            || reference_i16 != converted_i16[set]
            || reference_f32 != converted_f32[set]) {
            std::fputs("fused NEON correctness check failed\n", stderr);
            return 1;
        }
    }

    const double fused_i16_seconds = time_seconds([&] {
        for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
            const std::size_t set = iteration % buffer_sets;
            if (!unpack_neon_i16(packed[set], converted_i16[set])) {
                std::abort();
            }
        }
    });

    const double fused_f32_seconds = time_seconds([&] {
        for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
            const std::size_t set = iteration % buffer_sets;
            if (!unpack_neon_f32(packed[set], converted_f32[set])) {
                std::abort();
            }
        }
    });
#endif

    const double i16_seconds = time_seconds([&] {
        for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
            const std::size_t set = iteration % buffer_sets;
            if (airspy::driver::unpack_legacy_u12(packed[set], unpacked[set])
                    != airspy::driver::UnpackResult::ok
                || airspy::driver::convert_u12_to_i16(
                       unpacked[set], converted_i16[set])
                    != airspy::driver::ConversionResult::ok) {
                std::abort();
            }
        }
    });

    const double direct_i16_seconds = time_seconds([&] {
        for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
            const std::size_t set = iteration % buffer_sets;
            if (airspy::driver::unpack_legacy_u12_to_i16(
                    packed[set], converted_i16[set])
                != airspy::driver::UnpackResult::ok) {
                std::abort();
            }
        }
    });

    const double direct_f32_seconds = time_seconds([&] {
        for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
            const std::size_t set = iteration % buffer_sets;
            if (airspy::driver::unpack_legacy_u12_to_f32(
                    packed[set], converted_f32[set])
                != airspy::driver::UnpackResult::ok) {
                std::abort();
            }
        }
    });

    const double f32_seconds = time_seconds([&] {
        for (std::size_t iteration = 0; iteration < iterations; ++iteration) {
            const std::size_t set = iteration % buffer_sets;
            if (airspy::driver::unpack_legacy_u12(packed[set], unpacked[set])
                    != airspy::driver::UnpackResult::ok
                || airspy::driver::convert_u12_to_f32(
                       unpacked[set], converted_f32[set])
                    != airspy::driver::ConversionResult::ok) {
                std::abort();
            }
        }
    });

    std::uint64_t checksum = 0;
    for (std::size_t set = 0; set < buffer_sets; ++set) {
        checksum += unpacked[set][set];
        checksum += static_cast<std::uint16_t>(converted_i16[set][set]);
        checksum += static_cast<std::uint64_t>(
            converted_f32[set][set] * 2048.0F + 4096.0F);
    }

    const double total_samples =
        static_cast<double>(iterations * output_samples);
    const auto report = [total_samples](const char* label, double seconds) {
        std::printf(
            "%-20s %8.3f ms  %8.1f MS/s  %6.3f ns/sample\n",
            label,
            seconds * 1000.0,
            total_samples / seconds / 1.0e6,
            seconds * 1.0e9 / total_samples);
    };

    report("unpack u12 -> u16", unpack_seconds);
#if defined(__aarch64__)
    report("NEON u12 -> u16", neon_seconds);
#endif
    report("unpack + i16", i16_seconds);
    report("unpack + f32", f32_seconds);
    report("direct packed i16", direct_i16_seconds);
    report("direct packed f32", direct_f32_seconds);
#if defined(__aarch64__)
    report("fused NEON i16", fused_i16_seconds);
    report("fused NEON f32", fused_f32_seconds);
#endif
    std::printf("checksum             %llu\n",
                static_cast<unsigned long long>(checksum));
}
