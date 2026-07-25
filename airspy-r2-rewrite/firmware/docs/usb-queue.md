# Buffer and USB queue refactor

The stock main-M0 loop receives a wrapping offset from the M4 and calls the
blocking USB scheduler. Bulk IN owns one dTD. If that descriptor is occupied,
the M0 retries allocation in a tight loop until completion returns it.

That compact mechanism conflates four different facts:

1. which sample bank DMA may write;
2. which completed bank is ready to publish;
3. which dTD names that bank;
4. whether the endpoint is idle, priming, or consuming a chain.

The executable refactor separates those facts. A buffer walks through named
ownership states, while a dTD separately walks through `FREE`, `LINKED`, and
`ACTIVE`. The dTD records the buffer index, stream generation, byte count, and
first sample sequence. Validation rejects duplicate buffer references, broken
chains, stale generations, and any USB-owned buffer without a descriptor.

## What changes at stock depth

With two buffers and one dTD, descriptor exhaustion returns `WOULD_BLOCK`.
Nothing spins inside the allocator. The bounded transport pump submits at most
one pass over the buffer ring, reports that it is waiting for a descriptor, and
returns. The caller can return to `WFE`, report backpressure, or apply an
explicit overload policy. This preserves the one-descriptor safety rule while
removing hidden busy waiting.

## What a deeper logical queue does

With four modeled buffers and three dTDs, three distinct completed banks can be
owned by USB. The first dTD primes an idle endpoint. Later dTDs append to the
tail. Completing the head releases only its buffer; the next dTD becomes active
without another logical prime. The retired descriptor can then be used to
append a newly completed bank.

The current test uses synthetic third and fourth addresses. It proves queue and
ownership behavior only. It does not prove that those addresses exist on the
Airspy board, are free in the linker layout, or are reachable by USB DMA.

## Accounting

The device snapshot distinguishes:

- the capture sequence extent;
- samples submitted to USB;
- samples retired by USB;
- samples explicitly declared lost;
- capture/descriptor backpressure events;
- queue high-water mark and configured depths;
- completion count and stream generation.

Contract v2 serializes this as the fixed `AST2` status record. The C++ host
rejects stale generations, regressing counters, and impossible
`retired > submitted > captured` order. Device loss and host-side delivery loss
remain separate.

## LPC4370 hardware qualification

The model remains separate from production target firmware. A flashable
fixture based on the connected Mini's exact `946184a` revision has now proved:

- main-M0 writes and USB0 DMA reads for two additional 16 KiB local-SRAM
  windows;
- a 64-byte-aligned four-dTD pool in a dedicated, bounded local-SRAM section;
- the live ChipIdea endpoint-active and endpoint-idle/re-prime ATDTW branches;
- exact payload integrity across 4,404 descriptors in the scaled run;
- correct preservation of the endpoint data toggle across application stream
  restarts.

The fixture also exposed a stock lifecycle error: reusing endpoint
initialization for a receiver pause reset only the device's data toggle. An
odd-packet test then lost the first 512-byte packet while every dTD still
reported success. The target branch separates configuration-boundary reset from
application-level resume.

Still required before production depth exceeds one:

- prove whether ADCHS/GPDMA can directly write the additional local-SRAM
  windows, or budget an explicit staging copy;
- connect dTD identity to buffer ownership and generation in target code;
- make publication barriers explicit and audit them at the machine-code level;
- inject stall, flush, short-transfer, transaction-error, and buffer-error
  cases and prove references return exactly once;
- stress a continuous ADC producer rather than finite synthetic batches;
- decide whether IOC belongs on every descriptor or only selected descriptors.

The intended sequence is therefore one dTD with nonblocking explicit ownership,
then measured additional memory, then additional dTD depth. Merely changing
`USB_DEFINE_QUEUE(..., 1)` to a larger number remains unsafe.
