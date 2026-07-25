# Airspy R2/Mini USB transport audit

## Scope and pinned source

This is about Airspy One hardware (R2 and Mini), not Airspy HF+.

- Host: [`airspy/airspyone_host`](https://github.com/airspy/airspyone_host), pinned
  here at `fc61ab6be57ed61f0e2bdd9c6dfae74cacef57d0` (2026-07-19).
- Firmware: [`airspy/airspyone_firmware`](https://github.com/airspy/airspyone_firmware),
  pinned here at `cf1a37440d40e4229e9b474077e9fdd56f4926b1` (2021-10-06).

The source shows a 20 Msps real ADC stream for the R2's 10 Msps complex-IQ mode.
Each ADC result contains 12 information bits. IQ conversion happens on the host;
the USB stream is the real 20 Msps sequence.

## Byte-rate accounting

| R2 transport representation | Calculation | Payload rate |
|---|---:|---:|
| 16-bit ADC containers (library default) | 20,000,000 × 2 B | 40.0 MB/s (320 Mb/s) |
| Existing 12-bit lossless packing | 20,000,000 × 12/8 B | 30.0 MB/s (240 Mb/s) |

The host declares a 12-bit sample resolution but defaults `packing_enabled` to
false and explicitly sends `airspy_set_packing(device, 0)` during open. See
[`airspy.c` lines 880-908](https://github.com/airspy/airspyone_host/blob/fc61ab6be57ed61f0e2bdd9c6dfae74cacef57d0/libairspy/src/airspy.c#L880-L908).
The supported packing switch is implemented at
[`airspy.c` lines 1902-1937](https://github.com/airspy/airspyone_host/blob/fc61ab6be57ed61f0e2bdd9c6dfae74cacef57d0/libairspy/src/airspy.c#L1902-L1937).

High-speed USB 2.0 bulk uses 512-byte maximum packets. The theoretical bulk
payload ceiling, if essentially the whole periodic schedule is free, is commonly
calculated as 13 transactions/microframe × 512 bytes × 8,000 microframes/s =
53.248 MB/s. The official [USB 2.0 specification archive](https://usb.org/documents?search=usb+2.0)
is the normative source for packet and scheduling rules.

That puts the two R2 modes at roughly 75.1% and 56.3% of the theoretical bulk
payload ceiling before host-controller, hub, protocol, and competing-device
effects. The unpacked default leaves only 13.25 MB/s of theoretical headroom;
the existing lossless packing increases that to 23.25 MB/s. This explains why a
dedicated USB 2.0 link is much more reliable at 10 Msps and why topology matters:
ports can share an upstream high-speed transaction schedule even when they have
separate connectors.

At that 53.248 MB/s ceiling, lossless 12-bit packing represents 35.499 million
real ADC samples/s or 17.749 Msps complex. A less brittle 48 MB/s engineering
target represents 32 million real samples/s or 16 Msps complex. The former is a
wire-limit stretch goal; it is not a promise that the R820T/R860 analog path can
provide the same alias-free RF bandwidth.

If utilization is defined as a percentage of the usable bulk-payload ceiling
rather than the 480 Mb/s signaling rate, 90% is 47.923 MB/s or 15.974 Msps
complex, effectively the same 16 Msps engineering target.

## Transfer type and queueing

The firmware correctly enumerates endpoint `0x81` as high-speed bulk with a
512-byte maximum packet, not isochronous. See
[`usb_descriptor.c` lines 133-145](https://github.com/airspy/airspyone_firmware/blob/cf1a37440d40e4229e9b474077e9fdd56f4926b1/airspy_m0/usb_descriptor.c#L133-L145).
Bulk is a defensible choice: it provides CRC/retry semantics and can consume all
unreserved bus time. Conventional high-bandwidth USB 2.0 isochronous tops out at
three 1,024-byte transactions per microframe, or 24.576 MB/s per endpoint, which
cannot carry even the existing 30 MB/s packed R2 stream.

The host side is deeply queued: 16 asynchronous libusb transfers of 262,144 bytes
are allocated by default, providing 4 MiB in flight. See
[`airspy.c` lines 242-259](https://github.com/airspy/airspyone_host/blob/fc61ab6be57ed61f0e2bdd9c6dfae74cacef57d0/libairspy/src/airspy.c#L242-L259)
and [`airspy.c` lines 880-884](https://github.com/airspy/airspyone_host/blob/fc61ab6be57ed61f0e2bdd9c6dfae74cacef57d0/libairspy/src/airspy.c#L880-L884).

The device side is much shallower. Bulk IN is compiled with a transfer pool of
exactly one descriptor:
[`usb_endpoint.c` lines 46-54](https://github.com/airspy/airspyone_firmware/blob/cf1a37440d40e4229e9b474077e9fdd56f4926b1/airspy_m0/usb_endpoint.c#L46-L54).
The M0 wakes for each completed producer buffer and calls the blocking scheduler:
[`airspy_m0.c` lines 232-243](https://github.com/airspy/airspyone_firmware/blob/cf1a37440d40e4229e9b474077e9fdd56f4926b1/airspy_m0/airspy_m0.c#L232-L243).
When no descriptor is free, that scheduler spins until one is returned:
[`usb_queue.c` lines 199-209](https://github.com/airspy/airspyone_firmware/blob/cf1a37440d40e4229e9b474077e9fdd56f4926b1/common/usb_queue.c#L199-L209).

This was a deliberate inherited safety choice, not evidence that the controller
cannot chain dTDs. Airspy's USB stack came from HackRF after
[`7920490f`](https://github.com/greatscottgadgets/hackrf/commit/7920490f1e0051c009637ec43d6dc9ecd071b217),
which reduced bulk pools from four to one because there were only two producer
buffers: one filling and one safe for USB. Queueing a second buffer could expose
incomplete data.

Airspy has the same zero-copy constraint in unpacked mode. Of its two 16 KiB banks,
one is being filled by ADC DMA, leaving only one complete bank for USB. When the
second bank completes, DMA immediately wraps to the first. A second outstanding
dTD would be unsafe if USB had not retired the first bank. Hardware dTD chaining
is supported and the firmware implements the ATDTW append procedure, but useful
queue depth requires at least a third independently owned buffer and a DMA ring
that will not overwrite USB-owned storage.

This does not look like descriptor-RAM economizing: the aligned transfer wrapper
is about 128 bytes on the 32-bit target, and the same stack uses four-entry pools
for control transfers. The inherited HackRF commit specifically identifies the
two-buffer ownership hazard as the reason for reducing the bulk pool.

The LPC4370 has capacity for the missing buffers. Above the smaller shared/control
allocations, its current linker uses the three available main-AHB 16 KiB regions
for the two capture banks and M0 code, but also exposes 128 KiB and 72 KiB local
SRAM slaves. UM10503 says all AHB masters,
including USB0 and GPDMA, can access embedded memory. A third and fourth bank can
therefore be reserved in those two local SRAM regions, subject to a final link-map
check and a full-rate contention test. See `FIRMWARE_OPTIMIZATION.md` for the
allocation analysis and alternatives.

## Recommended transport experiments

1. Keep 16-bit transport as the baseline while the existing 12-bit path is
   instrumented. Although packing lowers USB payload, its current 8 KiB in-place
   implementation adds 70 MB/s of M4 SRAM traffic and loses timing margin in
   reported hardware testing.
2. Keep one bulk-IN dTD with the existing two-bank capture ring. Add explicit
   DMA/USB owner states and detect reuse of a still-active USB bank.
3. Instrument producer-ready, endpoint-prime, completion, NAK, and dropped-buffer
   counts. Compare dedicated-root-port and shared-hub topologies at both 40 and
   30 MB/s.
4. Add a third or fourth DMA/USB-accessible 16 KiB buffer, then increase the dTD
   pool and exercise the existing append path. Never queue a filling buffer or let
   DMA enter an active/queued USB buffer.
5. Keep the 16 host transfers initially. They are not the shallow side of this
   pipeline.
6. Treat a USB descriptor/queue patch and an onboard entropy codec as separate
   experiments; otherwise improvements cannot be attributed correctly.

Explicit buffer ownership and additional storage must precede the queue-depth
change. These should also precede any new compression firmware. See
`FIRMWARE_OPTIMIZATION.md` for the full analysis.
