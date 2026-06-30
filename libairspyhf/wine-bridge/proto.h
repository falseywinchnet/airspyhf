/*
 * AirSpy HF Wine bridge - shared wire protocol.
 *
 * Two processes:
 *   - shim   : a PE x86_64 airspyhf.dll loaded by SDR# inside Wine. Implements
 *              the public airspyhf API by forwarding calls over TCP loopback.
 *   - helper : a native arm64 macOS process linking the real libairspyhf.
 *
 * Transport: TCP on 127.0.0.1. Two connection kinds per opened device:
 *   - a CONTROL connection: synchronous request/response RPC.
 *   - a DATA connection: helper pushes IQ frames to the shim while streaming.
 *
 * Everything is little-endian (both x86_64 and arm64 are LE), structs are
 * byte-packed, so the two ends can memcpy scalar args directly.
 */
#ifndef AIRSPYHF_BRIDGE_PROTO_H
#define AIRSPYHF_BRIDGE_PROTO_H

#include <stdint.h>

#define AHB_DEFAULT_PORT      53977
#define AHB_PORT_ENV          "AIRSPYHF_BRIDGE_PORT"

#define AHB_CHANNEL_CONTROL   0x4354524Cu /* 'CTRL' */
#define AHB_CHANNEL_DATA      0x44415441u /* 'DATA' */
#define AHB_DATA_FRAME_MAGIC  0x46524d31u /* 'FRM1' */

/* Opcodes: one per public airspyhf_* entry point the shim forwards. */
enum ahb_op {
    AHB_OP_LIB_VERSION = 1,
    AHB_OP_LIST_DEVICES,
    AHB_OP_OPEN,
    AHB_OP_OPEN_SN,
    AHB_OP_CLOSE,
    AHB_OP_GET_OUTPUT_SIZE,
    AHB_OP_START,
    AHB_OP_STOP,
    AHB_OP_IS_STREAMING,
    AHB_OP_IS_LOW_IF,
    AHB_OP_SET_FREQ,
    AHB_OP_SET_FREQ_DOUBLE,
    AHB_OP_SET_LIB_DSP,
    AHB_OP_GET_SAMPLERATES,
    AHB_OP_SET_SAMPLERATE,
    AHB_OP_SET_ATT,
    AHB_OP_GET_ATT_STEPS,
    AHB_OP_SET_BIAS_TEE,
    AHB_OP_GET_BIAS_TEE_COUNT,
    AHB_OP_GET_BIAS_TEE_NAME,
    AHB_OP_GET_CALIBRATION,
    AHB_OP_SET_CALIBRATION,
    AHB_OP_GET_VCTCXO_CALIBRATION,
    AHB_OP_SET_VCTCXO_CALIBRATION,
    AHB_OP_GET_FRONTEND_OPTIONS,
    AHB_OP_SET_FRONTEND_OPTIONS,
    AHB_OP_SET_OPTIMAL_IQ_CORRECTION_POINT,
    AHB_OP_IQ_BALANCER_CONFIGURE,
    AHB_OP_FLASH_CONFIGURATION,
    AHB_OP_BOARD_PARTID_SERIALNO_READ,
    AHB_OP_VERSION_STRING_READ,
    AHB_OP_SET_USER_OUTPUT,
    AHB_OP_SET_HF_AGC,
    AHB_OP_SET_HF_AGC_THRESHOLD,
    AHB_OP_SET_HF_ATT,
    AHB_OP_SET_HF_LNA
};

#pragma pack(push, 1)

/* CONTROL request: op + device id + inline argument blob of in_len bytes. */
typedef struct {
    uint32_t op;
    uint64_t dev;       /* device id from AHB_OP_OPEN; 0 for global ops */
    uint32_t in_len;
} ahb_req_hdr;

/* CONTROL response: return code + output blob of out_len bytes. */
typedef struct {
    int32_t  ret;
    uint32_t out_len;
} ahb_resp_hdr;

/* First 4 bytes a connection sends: AHB_CHANNEL_CONTROL or AHB_CHANNEL_DATA. */
/* A DATA connection then sends this hello to bind itself to a device. */
typedef struct {
    uint64_t dev;
} ahb_data_hello;

/* Header before each streamed IQ frame on the DATA connection.
 * Payload = sample_count * sizeof(airspyhf_complex_float_t) (= 8 bytes). */
typedef struct {
    uint32_t magic;          /* AHB_DATA_FRAME_MAGIC */
    uint32_t sample_count;
    uint64_t dropped_samples;
} ahb_data_hdr;

#pragma pack(pop)

#endif /* AIRSPYHF_BRIDGE_PROTO_H */
