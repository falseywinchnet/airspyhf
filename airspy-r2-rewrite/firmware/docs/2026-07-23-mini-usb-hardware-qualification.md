# Airspy Mini USB hardware qualification — 2026-07-23

## Result

The connected Airspy Mini can expose two additional 16 KiB buffers from the
LPC4370's local SRAM to USB0 DMA. A four-dTD bulk-IN queue works on silicon, and
both decisions in the ChipIdea ATDTW append algorithm were deliberately
exercised with exact payload verification.

The work also found a separate stock endpoint-lifecycle bug: receiver restart
uses an operation that resets only the device's bulk data toggle. If the prior
stream ended after an odd number of packets, the host discards the next
512-byte packet as a duplicate even though the device reports successful dTD
completion. Separating USB-boundary initialization from application-level
resume removed the loss in the reproducer.

This is a hardware-fixture result, not yet a production multi-buffer receiver.

## Device and recovery

- Device: Airspy Mini, USB VID:PID `1d50:60a1`
- Serial: `35AC63DC2D6ABB4F`
- Link: USB high speed
- Stock firmware: `AirSpy MINI v1.0.0-rc10-0-g946184a 2016-09-19`
- Target branch: `codex/buffer-usb-hwtest`, based on `946184a`
- Final fixture image SHA-256:
  `b9eceefa3904b266a76b59b37d35c268f0d19697dec776f90c982207b569fbba`
- Original 1 MiB SPI image SHA-256:
  `2c833dab584ae2cf4857babf4a6e4cd78c7994cffd661c4ffdd563c98a5189c7`
- Original first 64 KiB SHA-256:
  `8945c242dd4e5070434c2a24a8989927a21e6e18c03ba764b8f505a43082d437`

The flash was dumped twice and compared before testing. Only the first 64 KiB
was erased by the firmware writer; calibration/configuration areas outside it
were not targeted.

## SRAM visibility

The fixture fills each candidate with a changing deterministic 32-bit pattern,
places the address directly into an EP0 IN dTD, and asks the USB controller to
return all 16 KiB. The host verifies every word.

| Candidate | Address | Repetitions | Result |
|---|---:|---:|---|
| Known AHB capture bank | `0x20004000` | 1,000 | pass |
| Local SRAM1 | `0x10018000` | 1,000 | pass |
| Local SRAM2 | `0x10084000` | 1,000 | pass |

Each candidate transferred 15.62 MiB without a mismatch. This proves main-M0
write access and USB0 DMA read access. It does not prove direct ADCHS/GPDMA
write access to the two local banks.

The M4 map ended at `0x100043e0` in the initial fixture and has a hard assertion
that it remain below `0x10018000`. M4 BSS ended at `0x10080068` and is asserted
below `0x10084000`.

## Descriptor placement

Changing the bulk-IN pool from one to four initially overflowed the M0's
16 KiB code/data region by 768 bytes. That was a linker-layout limit, not a
chip-memory limit.

The four 64-byte-aligned transfer objects now occupy a dedicated 512-byte
`.usb_dma_metadata` section at `0x1001c000`. This lies after the local SRAM1
test buffer and below the end of the 128 KiB bank. The linker bounds the section
to `0x1001c000..0x1001ffff`.

## ATDTW test

The ordinary phase schedules a 16 KiB dTD, waits for `ENDPTPRIME` to clear, and
then appends three more 16 KiB dTDs. This forces the implementation through
ATDTW while `ENDPTSTAT` reports the endpoint active.

The race phase first leaves a host bulk read pending, schedules a 512-byte dTD,
waits until the controller consumes it while the completion interrupt is still
deferred, and appends the next dTD. The software queue still has the old tail,
but `ENDPTSTAT` is idle, so the ATDTW algorithm must prime the newly appended
dTD. Two more dTDs follow.

Final scaled run:

| Phase | Batches | dTDs | Appends | Active exits | Re-prime exits | Errors |
|---|---:|---:|---:|---:|---:|---:|
| Endpoint active | 1,000 | 4,000 | 3,000 | 3,000 | 0 | 0 |
| Forced endpoint idle | 101 | 404 | 303 | 101 | 101 | 0 |

All payload words matched. There were no ATDTW sampling retries, descriptor
errors, or scheduling failures. Every descriptor used IOC, so the observed
maximum retired by one completion callback was one.

## Data-toggle failure and fix

Before the lifecycle split, ten repeated open/claim/init tests with five
forced-race batches at the end alternated:

- cycles 1, 3, 5, 7, and 9 passed;
- cycles 2, 4, 6, 8, and 10 timed out at 65,024 of 65,536 bytes.

Failure telemetry said all four dTDs retired and the controller saw no error.
The exact one-packet loss and odd/even alternation identify a data-toggle
disagreement. The old endpoint initializer asserted the controller's TX toggle
reset whenever streaming was started. The host had seen no USB reset,
`SET_CONFIGURATION`, `SET_INTERFACE`, or endpoint-halt clear, so it correctly
preserved its toggle. It ACKed and discarded the device's first packet as a
duplicate.

The refactor gives the two operations distinct names and behavior:

- `usb_endpoint_init()` configures a USB boundary and resets the toggle;
- `usb_endpoint_resume()` reconfigures after an application pause without
  resetting the toggle.

After the change, all ten odd-packet restart cycles passed. A scaled run ending
with 101 odd-packet race batches passed, and a new host process immediately
completed another active/race run without losing a packet.

## Receiver sanity check

With the fixture firmware, the unmodified C host tool completed finite raw
captures at both advertised Mini rates:

- 6.0 MS/s: 12,000,000 requested samples, completed;
- 3.0 MS/s: 6,000,000 requested samples, completed.

This is only a regression smoke test. It does not measure sustained multi-buffer
ADC-to-USB throughput.

## Source documents

- NXP UM10503, section 23.10.5.4.3, especially the ATDTW add-dTD algorithm on
  PDF page 578.
- AN3631, the ChipIdea/Freescale device queue-head and dTD model.
- LPC4370 data sheet, AHB multilayer matrix diagram on PDF page 60.

## Next gate

The next target experiment should connect four explicit buffer-ownership
records to the four dTDs and drive them from a synthetic continuous producer,
then from the real ADC path. Before treating the local banks as capture buffers,
test GPDMA writes into both. Error injection and partial-transfer retirement
must precede a production queue-depth change.
