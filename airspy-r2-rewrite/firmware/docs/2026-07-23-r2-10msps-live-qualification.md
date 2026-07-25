# Airspy R2 live 10 MSPS qualification

Device: Airspy R2/NOS `35AC63DC2D7D704F`

## Recovery

The original 64 KiB flash was read twice before modification. Both reads were
identical.

- `hardware/recovery/r2-35AC63DC2D7D704F/stock-dump-a.bin`
- `hardware/recovery/r2-35AC63DC2D7D704F/stock-dump-b.bin`
- SHA-256:
  `8945c242dd4e5070434c2a24a8989927a21e6e18c03ba764b8f505a43082d437`

## Loaded candidate

- `hardware/images/airspy-ring-v3-universal-prepared-start-guard2-no-park.bin`
- SHA-256:
  `862783d2c797113f4d4734d91f34826bd1e4cb5bdb8c50bb4f8f911896b6aa31`

The image is universal: it selected the R2/NOS clock and samplerate tables on
this device and retains the Mini tables for Mini hardware. Cool park is not
enabled.

The matching host driver and firmware add a backward-compatible prepared
receiver start:

1. `RECEIVER_MODE_ARMED` raises the streaming clock and completes I2C/R820T
   setup while the ADC remains stopped, then enables an empty bulk endpoint.
2. The host submits its USB reads.
3. `RECEIVER_MODE_RX` initializes the ring and starts ADC/DMA.

Old host drivers may still use the original one-step RX request.

## Clock and throughput

The R2 selected PLL1 register `0x040608c0`, the native 140 MHz configuration.
At 10 MSPS the 16 KiB DMA banks completed in approximately 57,344 M4 cycles,
exactly matching 8,192 raw samples at 20 million raw samples/second.

A continuous 90-second run settled at approximately 20.0 million raw
samples/second. The DMA ISR cost was about 118 M4 cycles per bank.

## Startup diagnosis and final result

The first tests showed one or two FIFO overflows per finite capture. Moving the
Genesys hub directly to the Mac did not change them, disproving the initial
inference that an idle InstantView/display device was causing active bus
contention.

Timed telemetry showed that the count was established at startup and remained
unchanged through tens of thousands of banks. The cause was firmware ordering:
the M4 started ADC/DMA before the M0 performed high-speed I2C and R820T setup.
At 10 MSPS, that tuner work prevented the M0 from feeding USB dTDs long enough
to exhaust the ring.

The prepared-start protocol moves every tuner operation before ADC start.
After this change:

- ten of ten repeated 10 MSPS starts had zero ownership overwrites and zero
  ADC FIFO overflows
- a sustained live observation completed 84,430 banks with zero overwrite
  prevention, zero ownership overwrites, zero FIFO overflows, zero partial
  transfers, zero USB errors, and zero USB backpressure
- bank timing was 57,297--58,005 M4 cycles
- raw throughput settled at 20.0 million samples/second

The directly connected Genesys hub is sufficient for clean 10 MSPS operation.

## V4 constant-time queue follow-up

The next image,
`hardware/images/airspy-ring-v4-universal-o1-queue-no-park.bin`, retains the
qualified V3 wire contract and prepared-start behavior while changing the
internal USB queue representation:

- FIFO free-list and active-tail bookkeeping replace repeated list walks;
- ten transfer objects settle predictably onto the ten ADC ring buffers;
- the caller's PRIMASK state is restored after queue critical sections;
- M0/M4 cursors use increment-and-wrap rather than modulo;
- production telemetry work is compiled out of the hot path.

The first V4 build incorrectly cached all five ChipIdea dTD buffer pointers.
The controller advances buffer pointer zero as a transfer progresses, so a
descriptor reused after pool wrap retained a stale address. Length and
ownership counters could still balance while endpoint-zero payload was wrong.
A normal-firmware SPI-flash read exposed the defect: data first diverged around
byte 1025 and later reads contained zero-filled regions.

The corrected build restores buffer pointer zero from the submitted buffer on
every prepare and caches only immutable page pointers one through four.

- Corrected SHA-256:
  `1566bc5767bd21764af992a694a85925eaeb33987ee0d8257170942c949d167c`
- The independent RAM recovery loader programmed and read back all 20,012
  bytes exactly.
- Three read-only normal-firmware passes, using varying control-transfer
  lengths and many descriptor-pool wraps, matched all 20,012 bytes.
- Ten repeated unpacked 10 MS/s starts each balanced 553 produced, submitted,
  and retired banks without DMA, ownership, FIFO, partial-transfer, USB-error,
  or backpressure failures.
- A sustained explicitly unpacked run balanced 36,794 full 16 KiB banks
  (602,832,896 bytes), with zero active-run overwrite prevention and the same
  zero-error result.
- Full-bank completion timing was 57,324--57,999 M4 cycles.

The finite-stop cancellation count is teardown bookkeeping: the host stopped
with queued reads outstanding. It did not correspond to an active-stream
partial transfer, ordering failure, or lost ADC bank.
