/* End-to-end test: a Windows PE exe (run under Wine) that drives the shim DLL,
 * which forwards to the native helper and the real AirSpy. */
#include <stdio.h>
#include <windows.h>
#include "airspyhf.h"

static volatile long g_blocks = 0;
static volatile long g_samples = 0;

static int cb(airspyhf_transfer_t *t)
{
    g_blocks++;
    g_samples += t->sample_count;
    return 0;
}

int main(void)
{
    airspyhf_lib_version_t v;
    airspyhf_lib_version(&v);
    printf("lib_version: %u.%u.%u\n", v.major_version, v.minor_version, v.revision);

    uint64_t serials[8];
    int n = airspyhf_list_devices(serials, 8);
    printf("list_devices: %d device(s)\n", n);
    for (int i = 0; i < n && i < 8; i++)
        printf("   serial[%d] = 0x%016llx\n", i, (unsigned long long)serials[i]);

    if (n <= 0) { printf("no device present; transport OK, stopping here.\n"); return 0; }

    airspyhf_device_t *dev = NULL;
    if (airspyhf_open(&dev) != AIRSPYHF_SUCCESS || !dev) { printf("open FAILED\n"); return 1; }
    printf("open OK\n");

    char ver[128] = {0};
    if (airspyhf_version_string_read(dev, ver, sizeof(ver) - 1) == AIRSPYHF_SUCCESS)
        printf("firmware: %s\n", ver);

    uint32_t rates[16];
    int nr = airspyhf_get_samplerates(dev, rates, 0); /* returns count */
    printf("samplerate count: %d\n", nr);
    if (nr > 0 && nr <= 16 && airspyhf_get_samplerates(dev, rates, nr) == AIRSPYHF_SUCCESS) {
        for (int i = 0; i < nr; i++) printf("   rate[%d] = %u\n", i, rates[i]);
        airspyhf_set_samplerate(dev, rates[nr - 1]);
    }

    airspyhf_set_freq(dev, 10000000); /* 10 MHz */
    printf("output_size: %d samples/block\n", airspyhf_get_output_size(dev));

    printf("starting stream for ~2s...\n");
    if (airspyhf_start(dev, cb, NULL) != AIRSPYHF_SUCCESS) { printf("start FAILED\n"); airspyhf_close(dev); return 1; }
    printf("is_streaming: %d\n", airspyhf_is_streaming(dev));
    Sleep(2000);
    airspyhf_stop(dev);
    printf("stopped. blocks=%ld samples=%ld (~%ld kS/s)\n",
           g_blocks, g_samples, g_samples / 2000);

    airspyhf_close(dev);
    printf("closed. OK\n");
    return 0;
}
