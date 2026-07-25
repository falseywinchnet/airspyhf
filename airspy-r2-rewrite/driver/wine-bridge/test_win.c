#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <windows.h>
#include "airspy.h"

static volatile LONG blocks;
static volatile LONG64 samples;
static volatile LONG64 dropped;
static volatile LONG minimum_block = LONG_MAX;
static volatile LONG maximum_block;
static volatile LONG first_sample_type = -1;

static int callback(airspy_transfer_t *transfer)
{
    LONG count = transfer->sample_count;
    LONG observed;

    InterlockedIncrement(&blocks);
    InterlockedAdd64(&samples, transfer->sample_count);
    InterlockedAdd64(&dropped, transfer->dropped_samples);
    InterlockedCompareExchange(&first_sample_type, transfer->sample_type, -1);

    observed = minimum_block;
    while (count < observed &&
           InterlockedCompareExchange(&minimum_block, count, observed) != observed) {
        observed = minimum_block;
    }
    observed = maximum_block;
    while (count > observed &&
           InterlockedCompareExchange(&maximum_block, count, observed) != observed) {
        observed = maximum_block;
    }
    return 0;
}

int main(int argc, char **argv)
{
    unsigned run_ms = argc > 1 ? (unsigned)strtoul(argv[1], NULL, 10) * 1000u : 2000u;
    uint32_t requested_rate = argc > 2 ? (uint32_t)strtoul(argv[2], NULL, 10) : 0;
    enum airspy_sample_type requested_type = argc > 3
        ? (enum airspy_sample_type)strtoul(argv[3], NULL, 10)
        : AIRSPY_SAMPLE_FLOAT32_IQ;
    airspy_lib_version_t v = {0};
    airspy_lib_version(&v);
    printf("libairspy %u.%u.%u\n", v.major_version, v.minor_version, v.revision);

    uint64_t serials[8] = {0};
    int n = airspy_list_devices(serials, 8);
    printf("devices: %d\n", n);
    if (n <= 0) return 2;

    struct airspy_device *dev = NULL;
    int ret = airspy_open_sn(&dev, serials[0]);
    printf("open: %d\n", ret);
    if (ret != AIRSPY_SUCCESS) return 3;

    char fw[128] = {0};
    airspy_version_string_read(dev, fw, sizeof(fw));
    printf("firmware: %s\n", fw);

    uint32_t count = 0;
    airspy_get_samplerates(dev, &count, 0);
    uint32_t rates[16] = {0};
    if (count > 16) count = 16;
    airspy_get_samplerates(dev, rates, count);
    printf("rates:");
    for (uint32_t i = 0; i < count; ++i) printf(" %u", rates[i]);
    printf("\n");

    airspy_set_sample_type(dev, requested_type);
    if (!requested_rate) requested_rate = rates[0];
    ret = airspy_set_samplerate(dev, requested_rate);
    printf("set rate %u: %d\n", requested_rate, ret);
    airspy_set_freq(dev, 100000000);
    airspy_set_sensitivity_gain(dev, 10);
    ret = airspy_start_rx(dev, callback, NULL);
    printf("start: %d\n", ret);
    Sleep(run_ms);
    airspy_stop_rx(dev);
    printf("seconds=%.3f blocks=%ld samples=%lld rate=%.3f MS/s dropped=%lld "
           "block=[%ld,%ld] type=%ld\n",
           run_ms / 1000.0, blocks, samples,
           samples / (1000.0 * run_ms), dropped,
           minimum_block, maximum_block, first_sample_type);
    airspy_close(dev);
    return ret == AIRSPY_SUCCESS && blocks > 0 ? 0 : 4;
}
