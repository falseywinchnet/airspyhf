#include "airspy.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <thread>

namespace {

struct Capture {
    std::atomic<std::uint32_t> blocks{0};
    std::atomic<int> callback_stop_result{AIRSPY_SUCCESS};
    std::atomic<std::uint64_t> dropped_samples{0};
    std::uint32_t target{32};
    airspy_device* device{nullptr};
    bool stop_inside_callback{false};
    std::uint32_t callback_delay_ms{0};
};

int receive(airspy_transfer* const transfer)
{
    auto* const capture = static_cast<Capture*>(transfer->ctx);
    const std::uint32_t blocks =
        capture->blocks.fetch_add(1, std::memory_order_relaxed) + 1;
    capture->dropped_samples.fetch_add(
        transfer->dropped_samples,
        std::memory_order_relaxed);
    if (capture->callback_delay_ms != 0) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(capture->callback_delay_ms));
    }
    if (blocks < capture->target) {
        return 0;
    }
    if (capture->stop_inside_callback) {
        capture->callback_stop_result.store(
            airspy_stop_rx(capture->device),
            std::memory_order_relaxed);
        return 0;
    }
    return 1;
}

bool run_capture(
    airspy_device* const device,
    const std::uint64_t serial,
    const bool stop_inside_callback,
    const std::uint32_t callback_delay_ms = 0,
    const bool expect_host_drop = false)
{
    Capture capture;
    capture.device = device;
    capture.stop_inside_callback = stop_inside_callback;
    capture.callback_delay_ms = callback_delay_ms;
    if (callback_delay_ms != 0) {
        capture.target = 12;
    }
    int result = airspy_start_rx(device, receive, &capture);
    if (result != AIRSPY_SUCCESS) {
        std::fprintf(
            stderr,
            "start failed for %016llX: %d\n",
            static_cast<unsigned long long>(serial),
            result);
        return false;
    }

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (airspy_is_streaming(device) == AIRSPY_TRUE
           && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    const int stop_result = airspy_stop_rx(device);
    const std::uint32_t received =
        capture.blocks.load(std::memory_order_relaxed);
    const int callback_stop_result =
        capture.callback_stop_result.load(std::memory_order_relaxed);
    const std::uint64_t dropped_samples =
        capture.dropped_samples.load(std::memory_order_relaxed);
    std::printf(
        "%016llX mode=%s blocks=%u dropped=%llu callback-stop=%d final-stop=%d\n",
        static_cast<unsigned long long>(serial),
        stop_inside_callback
            ? "callback-stop"
            : (callback_delay_ms != 0 ? "slow-consumer" : "callback-return"),
        received,
        static_cast<unsigned long long>(dropped_samples),
        callback_stop_result,
        stop_result);
    std::fflush(stdout);
    return stop_result == AIRSPY_SUCCESS
        && callback_stop_result == AIRSPY_SUCCESS
        && received >= capture.target
        && (!expect_host_drop || dropped_samples != 0);
}

} // namespace

int main()
{
    std::uint64_t serials[8]{};
    const int count = airspy_list_devices(serials, 8);
    if (count <= 0) {
        std::fprintf(stderr, "no Airspy R2/Mini devices found: %d\n", count);
        return 1;
    }

    for (int index = 0; index < count; ++index) {
        airspy_device* device = nullptr;
        int result = airspy_open_sn(&device, serials[index]);
        if (result != AIRSPY_SUCCESS) {
            std::fprintf(stderr, "open failed: %d\n", result);
            return 2;
        }

        result = airspy_set_sample_type(device, AIRSPY_SAMPLE_UINT16_REAL);
        if (result == AIRSPY_SUCCESS) {
            result = airspy_set_samplerate(device, 0);
        }
        if (result != AIRSPY_SUCCESS) {
            std::fprintf(
                stderr,
                "configuration failed for %016llX: %d\n",
                static_cast<unsigned long long>(serials[index]),
                result);
            (void)airspy_close(device);
            return 3;
        }

        if (!run_capture(device, serials[index], false)
            || !run_capture(device, serials[index], true)
            || !run_capture(device, serials[index], false, 100, true)) {
            (void)airspy_close(device);
            return 4;
        }

        result = airspy_set_packing(device, 1);
        if (result != AIRSPY_SUCCESS
            || !run_capture(device, serials[index], false)) {
            std::fprintf(
                stderr,
                "packing transition failed for %016llX: %d\n",
                static_cast<unsigned long long>(serials[index]),
                result);
            (void)airspy_close(device);
            return 5;
        }

        constexpr airspy_sample_type packed_types[] = {
            AIRSPY_SAMPLE_INT16_REAL,
            AIRSPY_SAMPLE_FLOAT32_REAL,
            AIRSPY_SAMPLE_INT16_IQ,
            AIRSPY_SAMPLE_FLOAT32_IQ
        };
        for (const airspy_sample_type sample_type : packed_types) {
            result = airspy_set_sample_type(device, sample_type);
            if (result != AIRSPY_SUCCESS
                || !run_capture(device, serials[index], false)) {
                std::fprintf(
                    stderr,
                    "packed sample type %d failed for %016llX: %d\n",
                    static_cast<int>(sample_type),
                    static_cast<unsigned long long>(serials[index]),
                    result);
                (void)airspy_close(device);
                return 6;
            }
        }
        if (airspy_set_packing(device, 0) != AIRSPY_SUCCESS) {
            std::fprintf(
                stderr,
                "packing disable failed for %016llX\n",
                static_cast<unsigned long long>(serials[index]));
            (void)airspy_close(device);
            return 7;
        }

        result = airspy_close(device);
        if (result != AIRSPY_SUCCESS) {
            std::fprintf(stderr, "close failed: %d\n", result);
            return 8;
        }
    }
    return 0;
}
