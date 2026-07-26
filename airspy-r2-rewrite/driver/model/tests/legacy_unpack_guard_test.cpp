#include "airspy_driver/legacy_unpack.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <vector>

#if defined(_WIN32)
#include <windows.h>
#else
#include <sys/mman.h>
#include <unistd.h>
#endif

namespace {

std::uint16_t reference_sample(
    const std::uint8_t* const group,
    const std::size_t sample) noexcept
{
    const auto word = [group](const std::size_t offset) {
        return static_cast<std::uint32_t>(group[offset])
            | (static_cast<std::uint32_t>(group[offset + 1U]) << 8U)
            | (static_cast<std::uint32_t>(group[offset + 2U]) << 16U)
            | (static_cast<std::uint32_t>(group[offset + 3U]) << 24U);
    };
    const std::uint32_t first = word(0);
    const std::uint32_t second = word(4);
    const std::uint32_t third = word(8);
    switch (sample) {
    case 0: return static_cast<std::uint16_t>((first >> 20U) & 0xfffU);
    case 1: return static_cast<std::uint16_t>((first >> 8U) & 0xfffU);
    case 2: return static_cast<std::uint16_t>(
        ((first & 0xffU) << 4U) | ((second >> 28U) & 0xfU));
    case 3: return static_cast<std::uint16_t>((second >> 16U) & 0xfffU);
    case 4: return static_cast<std::uint16_t>((second >> 4U) & 0xfffU);
    case 5: return static_cast<std::uint16_t>(
        ((second & 0xfU) << 8U) | ((third >> 24U) & 0xffU));
    case 6: return static_cast<std::uint16_t>((third >> 12U) & 0xfffU);
    default: return static_cast<std::uint16_t>(third & 0xfffU);
    }
}

struct GuardedInput {
    std::uint8_t* allocation{};
    std::uint8_t* input{};
    std::size_t page_size{};

    ~GuardedInput()
    {
#if defined(_WIN32)
        if (allocation != nullptr) {
            VirtualFree(allocation, 0, MEM_RELEASE);
        }
#else
        if (allocation != nullptr) {
            munmap(allocation, page_size * 2U);
        }
#endif
    }
};

GuardedInput make_guarded_input(const std::size_t bytes)
{
    GuardedInput result;
#if defined(_WIN32)
    SYSTEM_INFO info{};
    GetSystemInfo(&info);
    result.page_size = info.dwPageSize;
    result.allocation = static_cast<std::uint8_t*>(VirtualAlloc(
        nullptr, result.page_size * 2U, MEM_RESERVE | MEM_COMMIT,
        PAGE_READWRITE));
    DWORD old_protection = 0;
    if (result.allocation != nullptr) {
        VirtualProtect(result.allocation + result.page_size,
            result.page_size, PAGE_NOACCESS, &old_protection);
    }
#else
    result.page_size = static_cast<std::size_t>(sysconf(_SC_PAGESIZE));
    result.allocation = static_cast<std::uint8_t*>(mmap(
        nullptr, result.page_size * 2U, PROT_READ | PROT_WRITE,
        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0));
    if (result.allocation == MAP_FAILED) {
        result.allocation = nullptr;
    } else {
        mprotect(result.allocation + result.page_size,
            result.page_size, PROT_NONE);
    }
#endif
    if (result.allocation != nullptr && bytes <= result.page_size) {
        result.input = result.allocation + result.page_size - bytes;
    }
    return result;
}

} // namespace

int main()
{
    constexpr std::size_t groups = 257U;
    constexpr std::size_t input_bytes = groups * 12U;
    GuardedInput guarded = make_guarded_input(input_bytes);
    if (guarded.input == nullptr) {
        std::fputs("could not allocate guarded input\n", stderr);
        return 1;
    }

    std::uint32_t state = 0x13579bdfU;
    for (std::size_t index = 0; index < input_bytes; ++index) {
        state = state * 1664525U + 1013904223U;
        guarded.input[index] = static_cast<std::uint8_t>(state >> 24U);
    }

    const std::span<const std::uint8_t> input(
        guarded.input, input_bytes);
    std::vector<std::uint16_t> u16(groups * 8U);
    std::vector<std::int16_t> i16(groups * 8U);
    std::vector<float> f32(groups * 8U);
    using namespace airspy::driver;
    if (unpack_legacy_u12(input, u16) != UnpackResult::ok
        || unpack_legacy_u12_to_i16(input, i16) != UnpackResult::ok
        || unpack_legacy_u12_to_f32(input, f32) != UnpackResult::ok) {
        std::fputs("unpacker rejected valid guarded input\n", stderr);
        return 1;
    }

    for (std::size_t group = 0; group < groups; ++group) {
        for (std::size_t lane = 0; lane < 8U; ++lane) {
            const std::size_t index = group * 8U + lane;
            const std::uint16_t expected =
                reference_sample(guarded.input + group * 12U, lane);
            const std::int16_t expected_i16 = static_cast<std::int16_t>(
                (static_cast<int>(expected) - 2048) * 16);
            const float expected_f32 =
                static_cast<float>(static_cast<int>(expected) - 2048)
                * (1.0F / 2048.0F);
            if (u16[index] != expected
                || i16[index] != expected_i16
                || f32[index] != expected_f32) {
                std::fprintf(stderr,
                    "mismatch at group %zu lane %zu\n", group, lane);
                return 1;
            }
        }
    }
    return 0;
}
