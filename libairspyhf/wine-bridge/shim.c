/*
 * AirSpy HF Wine bridge - PE shim (x86_64 airspyhf.dll for Wine).
 *
 * Implements the public airspyhf_* API by forwarding every call over TCP
 * loopback to the native macOS helper (helper.c). SDR# loads this exactly as
 * it would the real airspyhf.dll. The IQ stream is delivered on a separate
 * data connection; a Win32 thread (so it is Wine-managed) reads frames and
 * invokes SDR#'s callback.
 *
 * Build: see build.sh (x86_64-w64-mingw32-gcc, -lws2_32).
 */
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "airspyhf.h"
#include "proto.h"

struct airspyhf_device {
    SOCKET            ctrl;       /* persistent control connection */
    CRITICAL_SECTION  lock;       /* serialize control RPCs */
    uint64_t          id;         /* helper-side device id */

    airspyhf_sample_block_cb_fn cb;
    void             *ctx;

    SOCKET            data;       /* data connection during streaming */
    HANDLE            reader;     /* reader thread */
    volatile LONG     streaming;

    airspyhf_complex_float_t *outbuf;
    int               outbuf_cap; /* in samples */
};

/* ------------------------------------------------------------------ */
/* winsock plumbing                                                    */
/* ------------------------------------------------------------------ */

static volatile LONG g_wsa_done = 0;

static void ensure_wsa(void)
{
    if (InterlockedCompareExchange(&g_wsa_done, 1, 0) == 0) {
        WSADATA w;
        WSAStartup(MAKEWORD(2, 2), &w);
    }
}

static int bridge_port(void)
{
    char buf[16];
    DWORD n = GetEnvironmentVariableA(AHB_PORT_ENV, buf, sizeof(buf));
    if (n > 0 && n < sizeof(buf)) { int p = atoi(buf); if (p > 0) return p; }
    return AHB_DEFAULT_PORT;
}

static int send_full(SOCKET s, const void *buf, int n)
{
    const char *p = (const char *)buf;
    while (n > 0) {
        int r = send(s, p, n, 0);
        if (r == SOCKET_ERROR) return -1;
        p += r; n -= r;
    }
    return 0;
}

static int recv_full(SOCKET s, void *buf, int n)
{
    char *p = (char *)buf;
    while (n > 0) {
        int r = recv(s, p, n, 0);
        if (r == 0 || r == SOCKET_ERROR) return -1;
        p += r; n -= r;
    }
    return 0;
}

/* Open a connection and send its channel tag. */
static SOCKET dial(uint32_t channel)
{
    ensure_wsa();
    SOCKET s = socket(AF_INET, SOCK_STREAM, 0);
    if (s == INVALID_SOCKET) return INVALID_SOCKET;

    struct sockaddr_in a; memset(&a, 0, sizeof(a));
    a.sin_family = AF_INET;
    a.sin_port = htons((u_short)bridge_port());
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (connect(s, (struct sockaddr *)&a, sizeof(a)) != 0) { closesocket(s); return INVALID_SOCKET; }

    int one = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, (const char *)&one, sizeof(one));

    if (send_full(s, &channel, sizeof(channel)) != 0) { closesocket(s); return INVALID_SOCKET; }
    return s;
}

/* One request/response on a connected control socket. Returns the remote
 * return code, copies up to out_cap bytes of output into out. */
static int rpc_on(SOCKET s, uint32_t op, uint64_t dev,
                  const void *in, uint32_t in_len,
                  void *out, uint32_t out_cap)
{
    ahb_req_hdr req; req.op = op; req.dev = dev; req.in_len = in_len;
    if (send_full(s, &req, sizeof(req)) != 0) return AIRSPYHF_ERROR;
    if (in_len && send_full(s, in, (int)in_len) != 0) return AIRSPYHF_ERROR;

    ahb_resp_hdr resp;
    if (recv_full(s, &resp, sizeof(resp)) != 0) return AIRSPYHF_ERROR;

    uint32_t take = resp.out_len < out_cap ? resp.out_len : out_cap;
    if (take && recv_full(s, out, (int)take) != 0) return AIRSPYHF_ERROR;
    /* drain any remainder we didn't have room for */
    uint32_t rest = resp.out_len - take;
    while (rest) {
        char tmp[256];
        uint32_t c = rest < sizeof(tmp) ? rest : sizeof(tmp);
        if (recv_full(s, tmp, (int)c) != 0) return AIRSPYHF_ERROR;
        rest -= c;
    }
    return resp.ret;
}

/* Control RPC bound to an opened device (thread-safe). */
static int dev_rpc(airspyhf_device_t *d, uint32_t op,
                   const void *in, uint32_t in_len, void *out, uint32_t out_cap)
{
    if (!d) return AIRSPYHF_ERROR;
    EnterCriticalSection(&d->lock);
    int r = rpc_on(d->ctrl, op, d->id, in, in_len, out, out_cap);
    LeaveCriticalSection(&d->lock);
    return r;
}

/* Control RPC with no device, over a throwaway connection. */
static int global_rpc(uint32_t op, const void *in, uint32_t in_len, void *out, uint32_t out_cap)
{
    SOCKET s = dial(AHB_CHANNEL_CONTROL);
    if (s == INVALID_SOCKET) return AIRSPYHF_ERROR;
    int r = rpc_on(s, op, 0, in, in_len, out, out_cap);
    closesocket(s);
    return r;
}

/* ------------------------------------------------------------------ */
/* streaming                                                           */
/* ------------------------------------------------------------------ */

static DWORD WINAPI reader_thread(LPVOID arg)
{
    airspyhf_device_t *d = (airspyhf_device_t *)arg;
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);

    while (InterlockedCompareExchange(&d->streaming, 1, 1)) {
        ahb_data_hdr h;
        if (recv_full(d->data, &h, sizeof(h)) != 0) break;
        if (h.magic != AHB_DATA_FRAME_MAGIC) break;

        int sc = (int)h.sample_count;
        if (sc > d->outbuf_cap) {
            airspyhf_complex_float_t *nb =
                (airspyhf_complex_float_t *)realloc(d->outbuf, (size_t)sc * sizeof(*nb));
            if (!nb) break;
            d->outbuf = nb; d->outbuf_cap = sc;
        }
        if (recv_full(d->data, d->outbuf, sc * (int)sizeof(airspyhf_complex_float_t)) != 0) break;

        airspyhf_transfer_t t;
        t.device          = d;
        t.ctx             = d->ctx;
        t.samples         = d->outbuf;
        t.sample_count    = sc;
        t.dropped_samples = h.dropped_samples;

        if (d->cb && d->cb(&t) != 0) { InterlockedExchange(&d->streaming, 0); break; }
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* public API                                                          */
/* ------------------------------------------------------------------ */

void ADDCALL airspyhf_lib_version(airspyhf_lib_version_t *v)
{
    if (!v) return;
    if (global_rpc(AHB_OP_LIB_VERSION, NULL, 0, v, sizeof(*v)) != AIRSPYHF_SUCCESS) {
        v->major_version = AIRSPYHF_VER_MAJOR;
        v->minor_version = AIRSPYHF_VER_MINOR;
        v->revision      = AIRSPYHF_VER_REVISION;
    }
}

int ADDCALL airspyhf_list_devices(uint64_t *serials, int count)
{
    if (serials && count > 0)
        return global_rpc(AHB_OP_LIST_DEVICES, &count, sizeof(count),
                          serials, (uint32_t)count * sizeof(uint64_t));
    int zero = 0;
    return global_rpc(AHB_OP_LIST_DEVICES, &zero, sizeof(zero), NULL, 0);
}

static int open_common(airspyhf_device_t **out_dev, uint32_t op, const void *in, uint32_t in_len)
{
    if (!out_dev) return AIRSPYHF_ERROR;
    SOCKET s = dial(AHB_CHANNEL_CONTROL);
    if (s == INVALID_SOCKET) return AIRSPYHF_ERROR;

    uint64_t id = 0;
    int ret = rpc_on(s, op, 0, in, in_len, &id, sizeof(id));
    if (ret != AIRSPYHF_SUCCESS) { closesocket(s); return ret; }

    airspyhf_device_t *d = (airspyhf_device_t *)calloc(1, sizeof(*d));
    if (!d) { closesocket(s); return AIRSPYHF_ERROR; }
    d->ctrl = s;
    d->id   = id;
    d->data = INVALID_SOCKET;
    InitializeCriticalSection(&d->lock);
    *out_dev = d;
    return AIRSPYHF_SUCCESS;
}

int ADDCALL airspyhf_open(airspyhf_device_t **device)
{
    return open_common(device, AHB_OP_OPEN, NULL, 0);
}

int ADDCALL airspyhf_open_sn(airspyhf_device_t **device, uint64_t serial_number)
{
    return open_common(device, AHB_OP_OPEN_SN, &serial_number, sizeof(serial_number));
}

int ADDCALL airspyhf_open_fd(airspyhf_device_t **device, int fd)
{
    (void)device; (void)fd;
    return AIRSPYHF_UNSUPPORTED; /* a host fd cannot cross the Wine/native boundary */
}

int ADDCALL airspyhf_stop(airspyhf_device_t *device)
{
    if (!device) return AIRSPYHF_ERROR;
    int ret = dev_rpc(device, AHB_OP_STOP, NULL, 0, NULL, 0);
    InterlockedExchange(&device->streaming, 0);
    if (device->data != INVALID_SOCKET) {
        shutdown(device->data, SD_BOTH);
        closesocket(device->data);
        device->data = INVALID_SOCKET;
    }
    if (device->reader) {
        WaitForSingleObject(device->reader, 2000);
        CloseHandle(device->reader);
        device->reader = NULL;
    }
    return ret;
}

int ADDCALL airspyhf_close(airspyhf_device_t *device)
{
    if (!device) return AIRSPYHF_ERROR;
    if (InterlockedCompareExchange(&device->streaming, 1, 1)) airspyhf_stop(device);
    int ret = dev_rpc(device, AHB_OP_CLOSE, NULL, 0, NULL, 0);
    closesocket(device->ctrl);
    DeleteCriticalSection(&device->lock);
    free(device->outbuf);
    free(device);
    return ret;
}

int ADDCALL airspyhf_start(airspyhf_device_t *device, airspyhf_sample_block_cb_fn callback, void *ctx)
{
    if (!device) return AIRSPYHF_ERROR;
    device->cb  = callback;
    device->ctx = ctx;

    int ret = dev_rpc(device, AHB_OP_START, NULL, 0, NULL, 0);
    if (ret != AIRSPYHF_SUCCESS) return ret;

    device->data = dial(AHB_CHANNEL_DATA);
    if (device->data == INVALID_SOCKET) { dev_rpc(device, AHB_OP_STOP, NULL, 0, NULL, 0); return AIRSPYHF_ERROR; }
    ahb_data_hello hello; hello.dev = device->id;
    if (send_full(device->data, &hello, sizeof(hello)) != 0) {
        closesocket(device->data); device->data = INVALID_SOCKET;
        dev_rpc(device, AHB_OP_STOP, NULL, 0, NULL, 0);
        return AIRSPYHF_ERROR;
    }

    InterlockedExchange(&device->streaming, 1);
    device->reader = CreateThread(NULL, 0, reader_thread, device, 0, NULL);
    if (!device->reader) { airspyhf_stop(device); return AIRSPYHF_ERROR; }
    return AIRSPYHF_SUCCESS;
}

int ADDCALL airspyhf_is_streaming(airspyhf_device_t *device)
{
    return dev_rpc(device, AHB_OP_IS_STREAMING, NULL, 0, NULL, 0);
}

/* ---- simple pass-throughs ---------------------------------------- */

int ADDCALL airspyhf_get_output_size(airspyhf_device_t *device)
{ return dev_rpc(device, AHB_OP_GET_OUTPUT_SIZE, NULL, 0, NULL, 0); }

int ADDCALL airspyhf_is_low_if(airspyhf_device_t *device)
{ return dev_rpc(device, AHB_OP_IS_LOW_IF, NULL, 0, NULL, 0); }

int ADDCALL airspyhf_set_freq(airspyhf_device_t *device, const uint32_t freq_hz)
{ return dev_rpc(device, AHB_OP_SET_FREQ, &freq_hz, sizeof(freq_hz), NULL, 0); }

int ADDCALL airspyhf_set_freq_double(airspyhf_device_t *device, const double freq_hz)
{ return dev_rpc(device, AHB_OP_SET_FREQ_DOUBLE, &freq_hz, sizeof(freq_hz), NULL, 0); }

int ADDCALL airspyhf_set_lib_dsp(airspyhf_device_t *device, const uint8_t flag)
{ return dev_rpc(device, AHB_OP_SET_LIB_DSP, &flag, sizeof(flag), NULL, 0); }

int ADDCALL airspyhf_get_samplerates(airspyhf_device_t *device, uint32_t *buffer, const uint32_t len)
{ return dev_rpc(device, AHB_OP_GET_SAMPLERATES, &len, sizeof(len),
                 buffer, (len ? len : 1) * sizeof(uint32_t)); }

int ADDCALL airspyhf_set_samplerate(airspyhf_device_t *device, uint32_t samplerate)
{ return dev_rpc(device, AHB_OP_SET_SAMPLERATE, &samplerate, sizeof(samplerate), NULL, 0); }

int ADDCALL airspyhf_set_att(airspyhf_device_t *device, float value)
{ return dev_rpc(device, AHB_OP_SET_ATT, &value, sizeof(value), NULL, 0); }

int ADDCALL airspyhf_get_att_steps(airspyhf_device_t *device, void *buffer, const uint32_t len)
{ return dev_rpc(device, AHB_OP_GET_ATT_STEPS, &len, sizeof(len),
                 buffer, (len ? len : 1) * sizeof(float)); }

int ADDCALL airspyhf_set_bias_tee(airspyhf_device_t *device, int8_t value)
{ return dev_rpc(device, AHB_OP_SET_BIAS_TEE, &value, sizeof(value), NULL, 0); }

int ADDCALL airspyhf_get_bias_tee_count(airspyhf_device_t *device, int32_t *count)
{ return dev_rpc(device, AHB_OP_GET_BIAS_TEE_COUNT, NULL, 0, count, sizeof(*count)); }

int ADDCALL airspyhf_get_bias_tee_name(airspyhf_device_t *device, int32_t index, char *version, uint8_t length)
{
    uint8_t in[sizeof(int32_t) + 1];
    memcpy(in, &index, sizeof(index));
    in[sizeof(index)] = length;
    return dev_rpc(device, AHB_OP_GET_BIAS_TEE_NAME, in, sizeof(in), version, length ? length : 1);
}

int ADDCALL airspyhf_get_calibration(airspyhf_device_t *device, int32_t *ppb)
{ return dev_rpc(device, AHB_OP_GET_CALIBRATION, NULL, 0, ppb, sizeof(*ppb)); }

int ADDCALL airspyhf_set_calibration(airspyhf_device_t *device, int32_t ppb)
{ return dev_rpc(device, AHB_OP_SET_CALIBRATION, &ppb, sizeof(ppb), NULL, 0); }

int ADDCALL airspyhf_get_vctcxo_calibration(airspyhf_device_t *device, uint16_t *vc)
{ return dev_rpc(device, AHB_OP_GET_VCTCXO_CALIBRATION, NULL, 0, vc, sizeof(*vc)); }

int ADDCALL airspyhf_set_vctcxo_calibration(airspyhf_device_t *device, uint16_t vc)
{ return dev_rpc(device, AHB_OP_SET_VCTCXO_CALIBRATION, &vc, sizeof(vc), NULL, 0); }

int ADDCALL airspyhf_get_frontend_options(airspyhf_device_t *device, uint32_t *flags)
{ return dev_rpc(device, AHB_OP_GET_FRONTEND_OPTIONS, NULL, 0, flags, sizeof(*flags)); }

int ADDCALL airspyhf_set_frontend_options(airspyhf_device_t *device, uint32_t flags)
{ return dev_rpc(device, AHB_OP_SET_FRONTEND_OPTIONS, &flags, sizeof(flags), NULL, 0); }

int ADDCALL airspyhf_set_optimal_iq_correction_point(airspyhf_device_t *device, float w)
{ return dev_rpc(device, AHB_OP_SET_OPTIMAL_IQ_CORRECTION_POINT, &w, sizeof(w), NULL, 0); }

int ADDCALL airspyhf_iq_balancer_configure(airspyhf_device_t *device, int buffers_to_skip, int fft_integration, int fft_overlap, int correlation_integration)
{
    int32_t a[4] = { buffers_to_skip, fft_integration, fft_overlap, correlation_integration };
    return dev_rpc(device, AHB_OP_IQ_BALANCER_CONFIGURE, a, sizeof(a), NULL, 0);
}

int ADDCALL airspyhf_flash_configuration(airspyhf_device_t *device)
{ return dev_rpc(device, AHB_OP_FLASH_CONFIGURATION, NULL, 0, NULL, 0); }

int ADDCALL airspyhf_board_partid_serialno_read(airspyhf_device_t *device, airspyhf_read_partid_serialno_t *r)
{ return dev_rpc(device, AHB_OP_BOARD_PARTID_SERIALNO_READ, NULL, 0, r, sizeof(*r)); }

int ADDCALL airspyhf_version_string_read(airspyhf_device_t *device, char *version, uint8_t length)
{ return dev_rpc(device, AHB_OP_VERSION_STRING_READ, &length, sizeof(length), version, length ? length : 1); }

int ADDCALL airspyhf_set_user_output(airspyhf_device_t *device, airspyhf_user_output_t pin, airspyhf_user_output_state_t value)
{
    int32_t a[2] = { (int32_t)pin, (int32_t)value };
    return dev_rpc(device, AHB_OP_SET_USER_OUTPUT, a, sizeof(a), NULL, 0);
}

int ADDCALL airspyhf_set_hf_agc(airspyhf_device_t *device, uint8_t flag)
{ return dev_rpc(device, AHB_OP_SET_HF_AGC, &flag, sizeof(flag), NULL, 0); }

int ADDCALL airspyhf_set_hf_agc_threshold(airspyhf_device_t *device, uint8_t flag)
{ return dev_rpc(device, AHB_OP_SET_HF_AGC_THRESHOLD, &flag, sizeof(flag), NULL, 0); }

int ADDCALL airspyhf_set_hf_att(airspyhf_device_t *device, uint8_t att_index)
{ return dev_rpc(device, AHB_OP_SET_HF_ATT, &att_index, sizeof(att_index), NULL, 0); }

int ADDCALL airspyhf_set_hf_lna(airspyhf_device_t *device, uint8_t flag)
{ return dev_rpc(device, AHB_OP_SET_HF_LNA, &flag, sizeof(flag), NULL, 0); }
