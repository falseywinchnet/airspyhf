# Airspy R2/Mini bridge for SDR# under Wine

This directory adapts the working Airspy HF Wine bridge to the distinct
`libairspy` ABI used by Airspy R2 and Mini.  It deliberately does not emulate
USB inside Wine:

```text
SDR# -> x86_64 PE airspy.dll -> TCP loopback -> native airspy-helper
     -> native libairspy/libusb -> Airspy Mini or R2
```

The control and sample streams use different sockets.  Every sample frame
states its sample type and exact payload byte count, so the bridge does not
confuse complex, real, unpacked, or packed raw transfers.

## Mini 3 MSPS compatibility correction

SDR# asks libairspy for real samples and presents half that rate as complex
bandwidth.  `airspy_get_samplerates()` therefore advertises the Mini's 6 and
3 MHz IQ rates as 12 and 6 MHz real rates.

The legacy setter first compares a by-value request against the undoubled IQ
list.  A request for the advertised 6 MHz real rate consequently collides with
the first 6 MHz IQ entry and selects index 0.  The radio returns 12 MHz real
while SDR# schedules DSP for 6 MHz, producing exactly the half-speed,
pitch-shifted audio observed at “3 MSPS Complex.”  The same ordering is present
in SDR#'s bundled stock DLL.

The helper resolves an advertised rate back to its list index before calling
the upstream setter.  The Windows probe verifies:

| SDR# path | Delivered rate | Drops |
|---|---:|---:|
| 3 MSPS complex / 6 MHz real | 6.003 MS/s | 0 |
| 6 MSPS complex / 12 MHz real | 12.006 MS/s | 0 |
| Native 3 MSPS complex | 3.002 MS/s | 0 |

Build with `./build.sh`, start `out/airspy-helper`, then put
`out/airspy.dll` beside `SDRSharp.exe`. The build compiles the release
`readable/` driver, links the helper directly against it, and copies that exact
artifact beside the helper as `libairspy.0.dylib`. The helper records the
resolved native-driver path at startup, so using an older system or upstream
libairspy is visible rather than inferred. The default loopback port is 53978;
set `AIRSPY_BRIDGE_PORT` on both processes to override it.

The build also produces `out/AirspyCounters.app`. It uses a read-only helper
operation to display the firmware's ADC, GPDMA, USB, ownership, and recovery
counters without opening the USB device a second time. The firmware counters
remain boot-lifetime evidence. The window establishes a new visual baseline
and shows zero whenever SDR# starts or stops the receiver or changes sample
rate. It stays above SDR# while open.

This bridge remains an investigation aid. It now dogfoods the readable C++
candidate while preserving the existing Windows shim ABI.
