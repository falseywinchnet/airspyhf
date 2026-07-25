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

## Universal V6a halt-free steering image

`airspy-ring-v6a-universal-isr-only-grants.bin` is the corrected experimental
R2/Mini image loaded onto R2 serial `35AC63DC2D7D704F` on 2026-07-25.

- Image size: 22,540 bytes
- SHA-256:
  `17b34a08e66ad21e1a829379fd5e14937d9d3a2eff7ebae04dd063b0303d9d6f`
- The first 22,540 flash bytes were read back and matched exactly before reset.
- After the firmware-owned reset request, the R2 re-enumerated at USB high
  speed and reported stream contract version 6 with clean idle counters.

V6a replaces V5c's ordinary-backpressure DMA halt with future-LLI steering over
the same ten qualified 16 KiB banks. M4 is the sole writer of a new USB-grant
generation, and the DMA ISR is the sole M4 context allowed to write it; M0 may
attach a dTD only after that grant. At least two distinct SRAM slave groups
remain outside USB ownership. Short congestion retains ordered banks; at the
floor, the oldest ungranted history is deliberately overwritten while
ADC/GPDMA continue.

The stream bytes and public Airspy API are unchanged. New private telemetry
counts deliberate steering discards separately from ownership violations,
along with alternation failures, no-candidate faults, stale completions,
minimum reusable depth, floor time, and worst steering cycles.

`airspy-ring-v6-broken-dual-context-grant.bin` is retained only as diagnostic
history and must not be deployed. It called the grant scan from both the DMA
ISR and the interruptible M4 main loop. The ISR could reserve the main loop's
selected bank before the interrupted main loop committed its older grant,
permanently removing that bank from the reusable pool without moving an error
counter.

## Universal V6b saturated-floor fast path

`airspy-ring-v6b-universal-floor-fast-path.bin` is the dual-radio contention
experiment for the R2 and Mini.

- Image size: 22,892 bytes
- SHA-256:
  `10fec0645b17d0d7c557b528843242d59289191fa6f170f747a78378e415035f`
- Stream contract version: 7

When only two non-USB-owned banks remain and no dTD has retired since the prior
boundary, the ISR now steers directly back into the just-completed bank. The
floor invariant already proves that it and the filling bank occupy different
SRAM slave ports. That saturated path therefore performs no ten-bank
destination scan, no grant scan, and no ten-bank telemetry-depth scan.

The cached condition is invalidated by the monotonic USB-retirement counter. A
retirement after the ISR's snapshot is detected on the next boundary, which
returns to the full scan/grant path and grows transport depth normally.
Telemetry separates floor-fast-path boundaries from full-scan boundaries so
the arbitration hypothesis can be evaluated against FIFO-overflow events.

Hardware rejected the intended performance effect: in a two-radio saturated
run only 3,516 of 209,956 boundaries used the shortcut, while 206,440 still
entered the scan path. The GUI's "minimum reusable banks" was also a sticky
low-water mark, not a live depth reading. V6b remains diagnostic history.

## Universal V6c retirement-mask steering image

`airspy-ring-v6c-universal-retirement-masks.bin` replaces V6b.

- Image size: 23,212 bytes
- SHA-256:
  `bf4af9de311c53f7af431b5ec45132d71bd01bdf448cdc0eeed9140cf4f8b7f8`
- Stream contract version: 8

M0 now publishes each completed USB retirement into a 16-entry SPSC
notification ring. M4 consumes those notifications at bank boundaries and
owns compact available/ready masks. Normal availability, floor, grant, and
free-destination decisions no longer discover ownership by rereading all ten
cross-core records. Selecting the oldest ready history still examines only
the ready candidates. A notification-ring overflow is explicit telemetry and
invokes the old ten-record reconstruction once as a recovery path.

The counter window now shows both current reusable depth and its sticky
minimum, plus retirement-notification overflows. `Full-scan boundaries` keeps
its existing ABI name but counts the general mask-based path in V6c; it no
longer means a ten-record ownership scan.

## Universal V7 GPDMA-margin image

`airspy-ring-v7-universal-gpdma-margin.bin` implements the reviewed
`NEXT_ADVANCEMENT.md` design for R2 and Mini.

- Image size: 23,596 bytes
- SHA-256:
  `784ab5910c28b83a011ca80b20a48a19ec1169b5c8683eb4b5046c78b53147e5`
- Stream contract version: 9
- Byte-for-byte flash readback passed on R2 `35AC63DC2D7D704F` and Mini
  `35AC63DC2D6ABB4F` before reset.
- Both rebooted at USB high speed and exposed the relocated version-9 contract
  with clean idle FIFO, DMA, ownership, grant-ring, retirement-ring, poison,
  and USB-system-error state.

V7 balances the ten banks 4/4/1/1 across SRAM slaves, relocates the shared
mailboxes and USB dTD pool away from their former hot slaves, caps retirement
and grant work at two entries per boundary, and replaces M0's grant search with
an M4-to-M0 SPSC notification ring. Available depth and group count are
incremental. Non-control diagnostic aggregation is compiled out of the DMA
boundary.

FIFO flush now respects the corrected empty/full semantics. Occupancy
high-water telemetry treats `LEVEL == 0` with `FIFO_EMPTY == 0` as sixteen
words. The first FIFO overflow halts ADC/GPDMA, suppresses the in-flight bank,
marks the epoch poisoned, and makes M0 answer one pending bulk read with a ZLP
so the unchanged host driver ends the stream. A subsequent RX command rebuilds
a fresh epoch. USB System Error and PLL1 AUTOBLOCK requirements from later
UM10503 revisions are also incorporated.

`airspy-ring-v7-universal-gpdma-margin-release.bin` is the explicit
`make RELEASE=1` build. `RELEASE=1` removes `-g` from M4, M0, and M0S source
compilation. Debug and boundary-aggregation feature macros remain disabled.
The release image is byte-for-byte identical to the qualified V7 image above,
including its size and SHA-256; debug information existed only in intermediate
ELF sections and never entered the SPI-flash binary.
