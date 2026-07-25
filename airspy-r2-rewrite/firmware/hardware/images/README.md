# Airspy Mini USB hardware-test firmware

`airspy-mini-946184a-four-dtd-toggle-fix-2026-07-23.bin` is the exact image
tested on Mini serial `35AC63DC2D6ABB4F`.

- Base firmware: Airspy Mini `946184a`, dated 2016-09-19
- Image size: 19,096 bytes
- SHA-256:
  `b9eceefa3904b266a76b59b37d35c268f0d19697dec776f90c982207b569fbba`
- Target branch: `codex/buffer-usb-hwtest`

This is an experimental hardware-qualification image, not a general release.
It includes:

- a four-entry bulk-IN dTD pool;
- explicit USB-boundary initialization versus toggle-preserving stream resume;
- two additional USB-DMA-visible local-SRAM test buffers;
- private vendor requests `0x80..0x83` for memory visibility and ATDTW
  telemetry/stress testing.

On the test Mini, 1,000 reads from each additional 16 KiB SRAM window passed.
A scaled run retired 4,404 dTDs with exact payloads and exercised both the
ATDTW endpoint-active and endpoint-idle/re-prime decisions. Ten odd-packet
restart cycles passed after the data-toggle lifecycle fix.

The full method and limitations are in
`../../docs/2026-07-23-mini-usb-hardware-qualification.md`.

## Universal V4 queue image

`airspy-ring-v4-universal-o1-queue-no-park.bin` is the corrected universal
R2/Mini image tested on R2 serial `35AC63DC2D7D704F`.

- Image size: 20,012 bytes
- SHA-256:
  `1566bc5767bd21764af992a694a85925eaeb33987ee0d8257170942c949d167c`
- No cool-park clock transition
- Packing remains optional and is disabled by default

Relative to the qualified V3 prepared-start image, V4 gives the USB queue FIFO
free-list and active-tail bookkeeping, constant-time append, stable mapping
between the ten transfer objects and ten ADC buffers, PRIMASK preservation,
cheaper ring cursors, and production-gated telemetry.

ChipIdea advances dTD buffer pointer zero during a transfer, so the corrected
image always restores that pointer when a descriptor is prepared. It caches
only immutable page pointers one through four. Three normal-firmware flash
readbacks and sustained unpacked 10 MS/s streaming passed after this fix.

See `../../docs/2026-07-23-r2-10msps-live-qualification.md`.

## Universal V5 recoverable-stream images

`airspy-ring-v5c-universal-alternating-sram.bin` is the current source-hardened
R2/Mini image loaded onto R2 serial `35AC63DC2D7D704F` on 2026-07-23.

- Image size: 21,196 bytes
- SHA-256:
  `138d64f1a3dd4007f813264f721435729b6180cd5ff0851baa36939586b888d5`
- The first 21,196 flash bytes were read back and matched exactly before reset.
- After reset the device enumerated at high speed and reported stream contract
  version 5 with clean idle ADC, GPDMA, USB queue, and ownership counters.
- A live 10 MS/s test advanced through thousands of 16 KiB banks with zero ADC
  FIFO, USB, DMA, ownership, recovery, and host-drop counters.

The earlier `v5` and `v5b` files are retained as diagnostic history. They
exposed one deterministic FIFO flag per ten-bank revolution because the final
two capture destinations shared local SRAM1. V5c alternates the five local
SRAM1 destinations with the five destinations on other SRAM slaves.

Long-duration streaming and fault-injection qualification remain pending. See
`../../docs/2026-07-23-hardening-pass-2-recoverable-streaming.md`.
