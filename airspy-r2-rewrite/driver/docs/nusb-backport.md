# AirspyHF nusb lessons to carry into Airspy One

The local AirspyHF Rust migration established several transport rules worth
reusing:

- workers do not borrow the public device object;
- a stream generation prevents stale workers affecting a restart;
- completed USB buffers are loaned to a bounded consumer queue and recycled;
- the USB worker never waits for DSP or an application callback;
- a timeout waiting for completion does not cancel a nusb transfer;
- shutdown requests cancellation and drains completions before storage dies;
- halt clearing occurs only after pending transfers retire;
- stop from inside the callback cannot join the callback thread;
- firmware-provided counts are bounded before allocation;
- wire bytes are decoded explicitly rather than through aligned native casts;
- Windows controls may need the claimed interface path.

Two HF details must not be copied:

- HF selects alternate setting 1; Airspy R2/Mini use interface 0, alt 0.
- HF transports native complex `i16`; Airspy One transports real unsigned
  12-bit ADC samples and performs real-to-IQ conversion on the host.

The Airspy One Rust port should use nusb only after the C++ implementation has
made cancellation, ownership, chunk assembly, callback stop, and fallback
behavior independently testable.
