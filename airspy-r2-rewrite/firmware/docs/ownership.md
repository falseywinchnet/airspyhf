# Buffer ownership

The stock design communicates progress through wrapping offsets. The readable
transcription must name that behavior before replacing it. The optimized model
uses monotonic sequence numbers and modulo only to choose a slot.

```text
FREE -> DMA_FILLING -> READY_RAW -> PROCESSING -> READY_USB
     -> USB_QUEUED -> USB_ACTIVE -> FREE
```

`PROCESSING` is optional. A legacy raw block can move directly from
`READY_RAW` to `READY_USB`.

Rules:

- exactly one state owns a slot;
- DMA enters only `FREE`;
- USB accepts only `READY_USB`;
- payload writes happen before a ready-state publication;
- USB retirement happens before return to `FREE`;
- each `USB_QUEUED` or `USB_ACTIVE` slot has exactly one live dTD;
- generation distinguishes an old stream from a restarted one;
- sequence counts capture order, not array position;
- every illegal transition is an error;
- overload policy is explicit.

“Nonblocking” does not mean “silently discard samples.” If no free slot exists,
the scheduler reports `WOULD_BLOCK`. A policy layer may stop capture, count and
drop a known range, or declare a fault. That decision must be observable and
must not be buried in the queue primitive.

When policy declares a drop, `airspy_buffer_record_drop()` advances the
monotonic sample sequence and the separate dropped-sample counter. The next
valid block therefore exposes the missing range rather than closing the gap and
pretending continuity.
