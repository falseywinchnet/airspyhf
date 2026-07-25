# airspy-r2-contract

This project owns facts shared by the Airspy R2/Mini firmware and host driver.
It does not contain USB backend code, DMA register code, DSP, or product policy.

## Contract v1: frozen legacy behavior

- USB VID/PID: `1d50:60a1`
- vendor interface 0, alternate setting 0
- bulk IN `0x81`, bulk OUT `0x02`
- EP0 maximum packet 64 bytes
- high-speed bulk maximum packet 512 bytes
- vendor requests 0 through 27
- request 26 selects raw or legacy lossless 12-bit packing
- unknown request numbers stall
- stream data has no header, sequence, epoch, or loss counter

Raw samples are unsigned 12-bit values stored in little-endian 16-bit words.
Legacy packing stores two unsigned 12-bit samples in three bytes. USB
completion boundaries are not sample-block or device-buffer boundaries.

## Contract v2: experimental and opt-in

The draft protocol-info request uses project-private request `0x80`. Existing
firmware rejects it because its dispatcher stalls requests above 27; the host
must interpret that rejection as “legacy v1 only.” A successful fixed-size
protocol-info reply points to length-delimited capability records, so additive
features do not require changing the bootstrap layout.

No v2 mode may alter the default v1 byte stream. Negotiation occurs while the
receiver is stopped and commits atomically. Every compressed block format must
have:

- exact reconstruction;
- an explicit raw fallback;
- a bounded worst-case encoded size;
- an integrity check;
- an epoch and monotonically increasing sequence;
- device-side loss accounting distinct from host-side loss.

The request number and layout remain marked experimental until verified on both
R2 and Mini hardware.

### Draft stream status

Request `0x84` returns a 64-byte `AST2` version-1 record. It reports one stream
generation and monotonically increasing captured, submitted, retired, dropped,
backpressure, and completion counts, plus configured buffer/descriptor depth
and queue high-water mark. Request `0x85` is reserved for an atomic clear while
the receiver is stopped.

The counters deliberately do not merge device loss with host transfer or
consumer-queue loss. A host must reject a status record from an old generation,
and a counter regression within one generation is a protocol fault.
