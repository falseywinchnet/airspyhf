zero guarantee of suitability. if you want the code, email.
should be more robust. tries to drop packets less. tries to fix a lot of quasi-bugs.
tries not to fall into stall or wedge conditions. tries to consume less memory.
NOT optimized for the packed form. only non-packed. 

Read counters with a USB vendor control transfer:
bmRequestType = 0xC0;   // device-to-host, vendor, device
bRequest      = 0x87;
wValue        = 0;
wIndex        = 0;
wLength       = 608;
The response is a little-endian, packed 608-byte ABI. Validate:
magic   == 0x53424F34
version == 5
It exposes:
Capture: completed/halted banks, overwrite prevention, ownership faults.
ADC: FIFO overflow, descriptor errors, overrange/underrange.
DMA: errors, status, recoveries, failed recoveries, estimated dropped banks.
USB: submitted/retired banks, backpressure, partial transfers, errors, cancellations.
USB lifecycle: suspend/resume, resets, port changes, controller-error IRQs.
Discontinuities: backpressure, suspend, partial-transfer events.
Timing: min/max bank completion and M4/M0 ISR/submit/retire cycle totals/maxima.
Ten per-buffer records: address, produced/submitted/retired generation, DMA timing, flags and retired bytes.
Clock configuration and transition counters.
Counters are unsigned 32-bit monotonic values and naturally wrap. Calculate deltas with unsigned subtraction. Reading is observational and safe during streaming. The reference ABI and reader are in the telemetry c file. 
