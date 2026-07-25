# Airspy Mini 120/144 MHz clock and ring qualification

The tested device was Airspy Mini `35AC63DC2D6ABB4F`. Both images retain the
ten-bank ADC ring and ten queued USB dTDs. The M4 is switched to the streaming
PLL setting on every receiver start and returned to the low-speed PLL setting
after every receiver stop.

## Recoverable images

- `hardware/images/airspy-mini-ring-v2-120mhz.bin`
  - SHA-256: `2a1e420d44c127c6d58e7cae96bfac67bbdfc8eb70ad2923fc6b00c6924bca70`
- `hardware/images/airspy-mini-ring-v2-144mhz.bin`
  - SHA-256: `0077cb64428f433eb51699f4a87049625f22c3509d4c6766be930cb86cf896c1`

The 144 MHz build is selected with `EXTRA_CFLAGS=-DAIRSPY_MINI_PLL1_144`.
It changes only the Mini high-speed PLL1 multiplier from 120 to 144 MHz and
the Mini high-speed R820T2 I2C divider from 150 to 180. The Mini low-speed
setting and all Airspy R2/NOS settings are unchanged.

## Results

The PLL register changed from `0x040408c0` at 120 MHz to `0x040508c0` at
144 MHz while streaming. Both returned to `0x040208c0` after stopping.

The ADC bank interval proves the effective M4 clock:

| Mode | 120 MHz | 144 MHz | Expected at 144 MHz |
| --- | ---: | ---: | ---: |
| 3 MSPS | about 163,840 cycles | about 196,608 cycles | 196,608 |
| 6 MSPS | about 81,920 cycles | about 98,304 cycles | 98,304 |

Twenty rapid alternating 3/6 MSPS captures at 144 MHz also retuned the tuner
across 91--148 MHz. All clock transitions and tuner operations succeeded.
Two short 6 MSPS runs recorded one ownership-overrun event during teardown.
The 120 MHz baseline showed the same event in two of ten rapid alternating
runs, so this is not evidence of 144 MHz PLL instability. It is associated
with short-capture USB cancellation/receiver-stop ordering and must be fixed
before raising the transport rate.

A sustained 144 MHz, 6 MSPS capture completed 11,767 full 16 KiB banks:

- zero ownership errors
- zero prevented overwrites
- zero partial USB retirements
- zero USB errors or backpressure
- completion interval 98,286--99,043 M4 cycles
- average/max M4 DMA ISR cost 116/137 cycles

The M4 telemetry is valid. The first M0 SysTick implementation produced zero
readings and therefore does not yet constitute a valid M0 cycle measurement.

## Interpretation

The Mini demonstrably operates its M4 and tuner-control path at 144 MHz and
returns to the lower clock after capture. This creates 20% more CPU-cycle
budget per fixed-duration ADC bank. It does not create USB bandwidth.

A future 12 MSPS complex-output mode would require twice the present raw
sample transport. With unpacked 16-bit samples this approaches the practical
limit of high-speed USB bulk transport, so 144 MHz makes the firmware side
more tangible but is not, by itself, proof that 12 MSPS can be sustained.
The next gate is deterministic receiver-stop ordering, followed by a
synthetic transport-only rate test before changing the ADC configuration.

No direct current or case-temperature sensor was available. Returning PLL1
to the low-speed state after stop is proven by the register telemetry; any
thermal claim beyond that requires an inline USB power meter and a temperature
probe or repeatable thermal-camera measurement.
