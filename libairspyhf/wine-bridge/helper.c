/*
 * AirSpy HF Wine bridge - native helper (macOS, arm64).
 *
 * Listens on TCP loopback, links the real libairspyhf, and exposes the whole
 * airspyhf_* API over the wire protocol in proto.h. The actual USB device is
 * owned here, in a normal native process, via libusb -> IOKit. The PE shim
 * inside Wine talks to this over 127.0.0.1.
 *
 * Build: see build.sh (clang, -lairspyhf -lpthread).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <signal.h>
#include <pthread.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>

#include <libairspyhf/airspyhf.h>
#include "proto.h"

#define MAX_DEVICES 8
#define LOGF(...) do { fprintf(stderr, "[helper] " __VA_ARGS__); fputc('\n', stderr); } while (0)

typedef struct {
    int             in_use;
    uint64_t        id;
    airspyhf_device_t *dev;
    int             data_fd;     /* -1 when no data connection bound */
    pthread_mutex_t wlock;       /* serialize/guard writes to data_fd */
} dev_rec;

static dev_rec       g_recs[MAX_DEVICES];
static pthread_mutex_t g_reg_lock = PTHREAD_MUTEX_INITIALIZER;
static uint64_t      g_next_id = 1;

/* ------------------------------------------------------------------ */
/* socket helpers                                                      */
/* ------------------------------------------------------------------ */

static int read_full(int fd, void *buf, size_t n)
{
    uint8_t *p = (uint8_t *)buf;
    while (n) {
        ssize_t r = recv(fd, p, n, 0);
        if (r == 0) return -1;             /* peer closed */
        if (r < 0) { if (errno == EINTR) continue; return -1; }
        p += r; n -= (size_t)r;
    }
    return 0;
}

static int write_full(int fd, const void *buf, size_t n)
{
    const uint8_t *p = (const uint8_t *)buf;
    while (n) {
        ssize_t r = send(fd, p, n, 0);
        if (r < 0) { if (errno == EINTR) continue; return -1; }
        p += r; n -= (size_t)r;
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* device registry                                                     */
/* ------------------------------------------------------------------ */

static dev_rec *rec_alloc(airspyhf_device_t *dev)
{
    pthread_mutex_lock(&g_reg_lock);
    dev_rec *r = NULL;
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (!g_recs[i].in_use) {
            r = &g_recs[i];
            r->in_use  = 1;
            r->id      = g_next_id++;
            r->dev     = dev;
            r->data_fd = -1;
            pthread_mutex_init(&r->wlock, NULL);
            break;
        }
    }
    pthread_mutex_unlock(&g_reg_lock);
    return r;
}

static dev_rec *rec_find(uint64_t id)
{
    pthread_mutex_lock(&g_reg_lock);
    dev_rec *r = NULL;
    for (int i = 0; i < MAX_DEVICES; i++) {
        if (g_recs[i].in_use && g_recs[i].id == id) { r = &g_recs[i]; break; }
    }
    pthread_mutex_unlock(&g_reg_lock);
    return r;
}

static void rec_free(dev_rec *r)
{
    pthread_mutex_lock(&g_reg_lock);
    if (r->data_fd >= 0) { close(r->data_fd); r->data_fd = -1; }
    pthread_mutex_destroy(&r->wlock);
    r->in_use = 0;
    r->dev = NULL;
    pthread_mutex_unlock(&g_reg_lock);
}

/* ------------------------------------------------------------------ */
/* streaming callback: runs on libairspyhf's consumer thread           */
/* ------------------------------------------------------------------ */

static int sample_cb(airspyhf_transfer_t *t)
{
    dev_rec *r = (dev_rec *)t->ctx;
    int rc = 0;
    pthread_mutex_lock(&r->wlock);
    if (r->data_fd >= 0) {
        ahb_data_hdr h;
        h.magic           = AHB_DATA_FRAME_MAGIC;
        h.sample_count    = (uint32_t)t->sample_count;
        h.dropped_samples = t->dropped_samples;
        size_t bytes = (size_t)t->sample_count * sizeof(airspyhf_complex_float_t);
        if (write_full(r->data_fd, &h, sizeof(h)) != 0 ||
            write_full(r->data_fd, t->samples, bytes) != 0) {
            /* shim's data connection went away: drop it, stop streaming */
            close(r->data_fd);
            r->data_fd = -1;
            rc = -1;
        }
    }
    pthread_mutex_unlock(&r->wlock);
    return rc;
}

/* ------------------------------------------------------------------ */
/* control dispatch                                                    */
/* ------------------------------------------------------------------ */

/* Reply with return code and an output blob. */
static int reply(int fd, int32_t ret, const void *out, uint32_t out_len)
{
    ahb_resp_hdr h; h.ret = ret; h.out_len = out_len;
    if (write_full(fd, &h, sizeof(h)) != 0) return -1;
    if (out_len && write_full(fd, out, out_len) != 0) return -1;
    return 0;
}

/* Handle one control connection until it closes. *owned tracks a device
 * opened on this connection so we can clean it up on disconnect. */
static void serve_control(int fd)
{
    dev_rec *owned = NULL;

    for (;;) {
        ahb_req_hdr req;
        if (read_full(fd, &req, sizeof(req)) != 0) break;

        uint8_t inbuf[512];
        if (req.in_len > sizeof(inbuf)) { LOGF("oversized req %u", req.in_len); break; }
        if (req.in_len && read_full(fd, inbuf, req.in_len) != 0) break;

        dev_rec *r = req.dev ? rec_find(req.dev) : NULL;
        airspyhf_device_t *d = r ? r->dev : NULL;
        int32_t ret = AIRSPYHF_ERROR;

        switch (req.op) {
        case AHB_OP_LIB_VERSION: {
            airspyhf_lib_version_t v; memset(&v, 0, sizeof(v));
            airspyhf_lib_version(&v);
            reply(fd, AIRSPYHF_SUCCESS, &v, sizeof(v));
            continue;
        }
        case AHB_OP_LIST_DEVICES: {
            int count; memcpy(&count, inbuf, sizeof(count));
            if (count > 0) {
                uint64_t *serials = (uint64_t *)calloc((size_t)count, sizeof(uint64_t));
                ret = airspyhf_list_devices(serials, count);
                reply(fd, ret, serials, (uint32_t)count * sizeof(uint64_t));
                free(serials);
            } else {
                ret = airspyhf_list_devices(NULL, 0);
                reply(fd, ret, NULL, 0);
            }
            continue;
        }
        case AHB_OP_OPEN:
        case AHB_OP_OPEN_SN: {
            airspyhf_device_t *nd = NULL;
            if (req.op == AHB_OP_OPEN) {
                ret = airspyhf_open(&nd);
            } else {
                uint64_t sn; memcpy(&sn, inbuf, sizeof(sn));
                ret = airspyhf_open_sn(&nd, sn);
            }
            if (ret == AIRSPYHF_SUCCESS && nd) {
                dev_rec *nr = rec_alloc(nd);
                if (!nr) { airspyhf_close(nd); reply(fd, AIRSPYHF_ERROR, NULL, 0); continue; }
                owned = nr;
                reply(fd, AIRSPYHF_SUCCESS, &nr->id, sizeof(nr->id));
            } else {
                reply(fd, ret, NULL, 0);
            }
            continue;
        }
        case AHB_OP_CLOSE:
            if (d) { airspyhf_stop(d); ret = airspyhf_close(d); }
            if (r) { if (owned == r) owned = NULL; rec_free(r); }
            reply(fd, ret, NULL, 0);
            continue;
        case AHB_OP_GET_OUTPUT_SIZE:
            ret = d ? airspyhf_get_output_size(d) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0);
            continue;
        case AHB_OP_START:
            ret = d ? airspyhf_start(d, sample_cb, r) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0);
            continue;
        case AHB_OP_STOP:
            ret = d ? airspyhf_stop(d) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0);
            continue;
        case AHB_OP_IS_STREAMING:
            ret = d ? airspyhf_is_streaming(d) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0);
            continue;
        case AHB_OP_IS_LOW_IF:
            ret = d ? airspyhf_is_low_if(d) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0);
            continue;
        case AHB_OP_SET_FREQ: {
            uint32_t hz; memcpy(&hz, inbuf, sizeof(hz));
            ret = d ? airspyhf_set_freq(d, hz) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0); continue;
        }
        case AHB_OP_SET_FREQ_DOUBLE: {
            double hz; memcpy(&hz, inbuf, sizeof(hz));
            ret = d ? airspyhf_set_freq_double(d, hz) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0); continue;
        }
        case AHB_OP_SET_LIB_DSP: {
            uint8_t f = inbuf[0];
            ret = d ? airspyhf_set_lib_dsp(d, f) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0); continue;
        }
        case AHB_OP_GET_SAMPLERATES: {
            uint32_t len; memcpy(&len, inbuf, sizeof(len));
            uint32_t n = len ? len : 1;
            uint32_t *buf = (uint32_t *)calloc(n, sizeof(uint32_t));
            ret = d ? airspyhf_get_samplerates(d, buf, len) : AIRSPYHF_ERROR;
            reply(fd, ret, buf, n * sizeof(uint32_t));
            free(buf); continue;
        }
        case AHB_OP_SET_SAMPLERATE: {
            uint32_t sr; memcpy(&sr, inbuf, sizeof(sr));
            ret = d ? airspyhf_set_samplerate(d, sr) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0); continue;
        }
        case AHB_OP_SET_ATT: {
            float v; memcpy(&v, inbuf, sizeof(v));
            ret = d ? airspyhf_set_att(d, v) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0); continue;
        }
        case AHB_OP_GET_ATT_STEPS: {
            uint32_t len; memcpy(&len, inbuf, sizeof(len));
            uint32_t n = len ? len : 1;
            float *buf = (float *)calloc(n, sizeof(float));
            ret = d ? airspyhf_get_att_steps(d, buf, len) : AIRSPYHF_ERROR;
            reply(fd, ret, buf, n * sizeof(float));
            free(buf); continue;
        }
        case AHB_OP_SET_BIAS_TEE: {
            int8_t v = (int8_t)inbuf[0];
            ret = d ? airspyhf_set_bias_tee(d, v) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0); continue;
        }
        case AHB_OP_GET_BIAS_TEE_COUNT: {
            int32_t c = 0;
            ret = d ? airspyhf_get_bias_tee_count(d, &c) : AIRSPYHF_ERROR;
            reply(fd, ret, &c, sizeof(c)); continue;
        }
        case AHB_OP_GET_BIAS_TEE_NAME: {
            int32_t index; uint8_t length;
            memcpy(&index, inbuf, sizeof(index));
            length = inbuf[sizeof(index)];
            char name[256]; memset(name, 0, sizeof(name));
            uint8_t cap = length ? length : 1;
            ret = d ? airspyhf_get_bias_tee_name(d, index, name, cap) : AIRSPYHF_ERROR;
            reply(fd, ret, name, cap); continue;
        }
        case AHB_OP_GET_CALIBRATION: {
            int32_t ppb = 0;
            ret = d ? airspyhf_get_calibration(d, &ppb) : AIRSPYHF_ERROR;
            reply(fd, ret, &ppb, sizeof(ppb)); continue;
        }
        case AHB_OP_SET_CALIBRATION: {
            int32_t ppb; memcpy(&ppb, inbuf, sizeof(ppb));
            ret = d ? airspyhf_set_calibration(d, ppb) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0); continue;
        }
        case AHB_OP_GET_VCTCXO_CALIBRATION: {
            uint16_t vc = 0;
            ret = d ? airspyhf_get_vctcxo_calibration(d, &vc) : AIRSPYHF_ERROR;
            reply(fd, ret, &vc, sizeof(vc)); continue;
        }
        case AHB_OP_SET_VCTCXO_CALIBRATION: {
            uint16_t vc; memcpy(&vc, inbuf, sizeof(vc));
            ret = d ? airspyhf_set_vctcxo_calibration(d, vc) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0); continue;
        }
        case AHB_OP_GET_FRONTEND_OPTIONS: {
            uint32_t flags = 0;
            ret = d ? airspyhf_get_frontend_options(d, &flags) : AIRSPYHF_ERROR;
            reply(fd, ret, &flags, sizeof(flags)); continue;
        }
        case AHB_OP_SET_FRONTEND_OPTIONS: {
            uint32_t flags; memcpy(&flags, inbuf, sizeof(flags));
            ret = d ? airspyhf_set_frontend_options(d, flags) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0); continue;
        }
        case AHB_OP_SET_OPTIMAL_IQ_CORRECTION_POINT: {
            float w; memcpy(&w, inbuf, sizeof(w));
            ret = d ? airspyhf_set_optimal_iq_correction_point(d, w) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0); continue;
        }
        case AHB_OP_IQ_BALANCER_CONFIGURE: {
            int32_t a[4]; memcpy(a, inbuf, sizeof(a));
            ret = d ? airspyhf_iq_balancer_configure(d, a[0], a[1], a[2], a[3]) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0); continue;
        }
        case AHB_OP_FLASH_CONFIGURATION:
            ret = d ? airspyhf_flash_configuration(d) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0); continue;
        case AHB_OP_BOARD_PARTID_SERIALNO_READ: {
            airspyhf_read_partid_serialno_t s; memset(&s, 0, sizeof(s));
            ret = d ? airspyhf_board_partid_serialno_read(d, &s) : AIRSPYHF_ERROR;
            reply(fd, ret, &s, sizeof(s)); continue;
        }
        case AHB_OP_VERSION_STRING_READ: {
            uint8_t length = inbuf[0];
            char ver[256]; memset(ver, 0, sizeof(ver));
            uint8_t cap = length ? length : 1;
            ret = d ? airspyhf_version_string_read(d, ver, cap) : AIRSPYHF_ERROR;
            reply(fd, ret, ver, cap); continue;
        }
        case AHB_OP_SET_USER_OUTPUT: {
            int32_t pin, val;
            memcpy(&pin, inbuf, sizeof(pin));
            memcpy(&val, inbuf + sizeof(pin), sizeof(val));
            ret = d ? airspyhf_set_user_output(d, (airspyhf_user_output_t)pin,
                                               (airspyhf_user_output_state_t)val) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0); continue;
        }
        case AHB_OP_SET_HF_AGC: {
            ret = d ? airspyhf_set_hf_agc(d, inbuf[0]) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0); continue;
        }
        case AHB_OP_SET_HF_AGC_THRESHOLD: {
            ret = d ? airspyhf_set_hf_agc_threshold(d, inbuf[0]) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0); continue;
        }
        case AHB_OP_SET_HF_ATT: {
            ret = d ? airspyhf_set_hf_att(d, inbuf[0]) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0); continue;
        }
        case AHB_OP_SET_HF_LNA: {
            ret = d ? airspyhf_set_hf_lna(d, inbuf[0]) : AIRSPYHF_ERROR;
            reply(fd, ret, NULL, 0); continue;
        }
        default:
            LOGF("unknown op %u", req.op);
            reply(fd, AIRSPYHF_ERROR, NULL, 0);
            continue;
        }
    }

    /* connection gone: tear down any device it owned */
    if (owned && owned->in_use) {
        if (owned->dev) { airspyhf_stop(owned->dev); airspyhf_close(owned->dev); }
        rec_free(owned);
    }
    close(fd);
}

static void serve_data(int fd)
{
    ahb_data_hello hello;
    if (read_full(fd, &hello, sizeof(hello)) != 0) { close(fd); return; }
    dev_rec *r = rec_find(hello.dev);
    if (!r) { LOGF("data conn for unknown dev %llu", (unsigned long long)hello.dev); close(fd); return; }

    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));

    pthread_mutex_lock(&r->wlock);
    if (r->data_fd >= 0) close(r->data_fd);
    r->data_fd = fd;
    pthread_mutex_unlock(&r->wlock);
    LOGF("data conn bound to dev %llu", (unsigned long long)hello.dev);

    /* Block until the shim closes the data socket; the sample callback owns
     * all writes. recv returning 0/<0 means the shim went away. */
    uint8_t scratch[64];
    for (;;) { ssize_t n = recv(fd, scratch, sizeof(scratch), 0);
               if (n <= 0) break; }

    pthread_mutex_lock(&r->wlock);
    if (r->data_fd == fd) r->data_fd = -1;
    pthread_mutex_unlock(&r->wlock);
    close(fd);
}

typedef struct { int fd; } conn_arg;

static void *conn_thread(void *arg)
{
    int fd = ((conn_arg *)arg)->fd;
    free(arg);
    uint32_t channel;
    if (read_full(fd, &channel, sizeof(channel)) != 0) { close(fd); return NULL; }
    if (channel == AHB_CHANNEL_CONTROL)      serve_control(fd);
    else if (channel == AHB_CHANNEL_DATA)    serve_data(fd);
    else { LOGF("bad channel tag 0x%08x", channel); close(fd); }
    return NULL;
}

int main(void)
{
    signal(SIGPIPE, SIG_IGN);

    int port = AHB_DEFAULT_PORT;
    const char *pe = getenv(AHB_PORT_ENV);
    if (pe) port = atoi(pe);

    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { perror("socket"); return 1; }
    int one = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in addr; memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = htons((uint16_t)port);
    if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) != 0) { perror("bind"); return 1; }
    if (listen(srv, 16) != 0) { perror("listen"); return 1; }

    {
        airspyhf_lib_version_t v; airspyhf_lib_version(&v);
        LOGF("listening on 127.0.0.1:%d  (libairspyhf %u.%u.%u)",
             port, v.major_version, v.minor_version, v.revision);
    }

    for (;;) {
        int fd = accept(srv, NULL, NULL);
        if (fd < 0) { if (errno == EINTR) continue; perror("accept"); break; }
        conn_arg *a = (conn_arg *)malloc(sizeof(conn_arg));
        a->fd = fd;
        pthread_t t;
        if (pthread_create(&t, NULL, conn_thread, a) != 0) { close(fd); free(a); continue; }
        pthread_detach(t);
    }
    close(srv);
    return 0;
}
