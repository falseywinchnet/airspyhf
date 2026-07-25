# Driver next advancement

Status: current engineering handoff  
Baseline: field-compatible C/libusb driver in `current/`  
Public compatibility: unchanged libairspy C API and ABI

## Why this is next

The existing driver is compact and field-proven across COTS hosts. Several
apparently crude choices have defensible intent:

- many large asynchronous transfers keep the host controller supplied;
- immediate resubmission isolates USB from application/DSP work;
- a separate consumer queue absorbs callback delay;
- ambiguous short transfers and transport errors fail fast rather than guessing
  whether stream order survived.

The next driver must not reinterpret those choices as accidental merely because
we can express a more elaborate design. The objective is:

> Fail slightly less often, slightly less abruptly, and with better evidence,
> while retaining conservative failure whenever ordering or endpoint epoch is
> genuinely uncertain.

The first implementation remains C++ over libusb. Rust/nusb is a possible later
port of the proven state machine, not the next redesign.

## Frozen public behavior

Stage one preserves:

- library names and exported C symbols;
- public structures, enum values, error codes, and calling convention;
- callback rules and `dropped_samples` field;
- device enumeration and serial selection;
- every legacy control request, direction, value, index, length, and fallback;
- raw, packed, real, and IQ sample interpretation;
- headerless bulk-IN byte stream;
- stop-from-callback behavior;
- compatibility with stock firmware and the current universal firmware.

That compatibility is one-directional and deliberately so. This driver runs
against old firmware, because the armed start falls back to a legacy one-step
`RECEIVER_MODE_RX`. Old drivers do **not** run against V7 firmware at 10 MSPS:
one-step start begins capture before the host has queued transfers, and the
eight-bank ring covers only 3.3 ms at that rate, so the device can poison its own
epoch during the host's startup. A receiver that cannot accept 40 MB/s from the
first sample is not supported.

Any path that restarts capture must therefore go through ARMED rather than
straight to RX, including sample-rate changes and stop/start cycles.

C++ exceptions never cross the C boundary.

## Frozen transport baseline

Do not tune this during readable tightening:

```text
16 asynchronous libusb bulk-IN transfers
256 KiB requested per transfer
8 consumer replacement buffers
24 allocated data buffers in the pointer-swap design
```

At 40 MB/s, the sixteen USB requests represent about 105 ms of outstanding
host work. The consumer queue represents another roughly 52 ms. This is large,
but it is the field-proven COTS baseline.

The USB callback currently:

1. receives a full completed transfer;
2. briefly locks the consumer queue;
3. swaps the transfer's buffer with an empty consumer buffer;
4. records pending host-queue drops;
5. signals the consumer;
6. immediately resubmits the same transfer object.

That division is intentional and remains.

## Short-transfer and drop semantics

A short USB transfer means:

```text
libusb status says COMPLETED
actual_length < requested_length
```

It does not mean merely that RF samples were lost. ADC overflow, firmware bank
discard, or a pause can still produce a later full 256 KiB host buffer: USB
only knows that the bytes it did transport passed packet integrity checks.

Likewise, when the host consumer queue is full, libairspy discards a completely
received 256 KiB block, increments its drop count, and resubmits the USB
transfer. That is not a short transfer.

Therefore:

- valid full USB buffers may contain an RF-time discontinuity and still
  continue;
- host queue overflow remains a counted full-buffer drop;
- an ambiguous short transfer, unknown endpoint epoch, or uncertain retirement
  remains fail-fast in the first hardened implementation;
- partial assembly must not blindly concatenate across stalls, resets, halt
  clearing, cancellation, or endpoint generations.

Known-valid backend segmentation may eventually use an internal byte assembler,
but only after its semantics are proven and without changing callback output.

## Stage A: readable C++ parity

Create explicit modules for:

```text
public C ABI boundary
device discovery and ownership
typed control requests
stream lifecycle and generation
libusb backend
transfer pool
consumer queue
sample conversion and IQ generation
error/accounting snapshot
```

This stage changes representation, not policy. Golden controls and sample
vectors must match the C baseline exactly.

The current prepared-start extension remains:

1. request `RECEIVER_MODE_ARMED`;
2. submit host bulk-IN transfers;
3. request `RECEIVER_MODE_RX`;
4. fall back to legacy one-step RX when old firmware stalls ARMED.

## Stage B: defensible tightening

After parity, make these internal changes without tuning transfer geometry:

- replace cross-thread `volatile` flags with defined atomics/state transitions;
- give every stream instance a generation so stale workers cannot affect a
  restart;
- make start transactional and unwind partially created threads/transfers;
- make stop/cancel asynchronous but observe one terminal completion for every
  submitted transfer before destroying its request or storage;
- allow stop from inside the application callback without self-join;
- separate transport status, host-queue drop, firmware discontinuity, and
  application stop reasons internally;
- classify stall, cancellation, disconnect, short completion, submission
  failure, and backend failure distinctly;
- clear an endpoint halt only after pending transfers are cancelled and drained;
- keep the USB completion path allocation-free and free of conversion,
  application callbacks, and routine logging;
- retain conservative termination when ordering or ownership cannot be proven.

Recovery is permitted only for a narrowly classified state whose cancellation,
retirement, endpoint toggle, and new epoch are all known. “Try to continue” is
not the default.

## Dedicated worker decision

The old driver already has the correct broad split:

- one libusb event/completion thread;
- one consumer thread for unpacking, IQ work, and the application callback.

The rewrite makes this ownership legible; it does not add threads merely to
claim higher priority. A userspace thread cannot grant bulk USB reserved
bandwidth. Its value is preventing DSP or client code from delaying transfer
resubmission.

## Buffer-reuse decision

libusb already provides the required reuse. The current driver recycles
`libusb_transfer` objects and swaps their data buffers with the consumer pool;
no per-transfer allocation or sample copy is required.

nusb ownership is not inherently faster. A future nusb port must preserve
separate capacity:

```text
16 buffers continuously available to the USB endpoint
8 consumer/replacement buffers
```

The current AirspyHF Rust implementation's shared eight-buffer ownership model
must not be copied into Airspy One; allowing consumer ownership to reduce
pending USB depth to zero is not parity with libairspy.

## Firmware counter use

The private firmware telemetry request `0x87` may be read by diagnostics or a
low-rate sidecar without changing the public API. It distinguishes ADC FIFO,
DMA, ownership, USB, backpressure, recovery, and host-side loss domains.

Do not silently redefine `dropped_samples`. Fold a firmware event into that
field only when its sample extent is known and the legacy meaning remains
honest. Otherwise retain it in internal diagnostics/telemetry.

Counter read failure never interrupts streaming.

## Tuning is a later stage

Sixteen 256 KiB requests are preserved until the tightened C++ driver is
dogfooded. No result from one Mac is called universal.

When tuning begins:

- treat 16 KiB as a natural firmware/backend quantum, not automatically the
  host request size;
- test only whole 16 KiB multiples;
- vary host request size, outstanding count, total outstanding bytes, consumer
  depth, and callback block size independently;
- test Windows, macOS, and Linux on ordinary COTS controllers and hubs;
- measure throughput, queue starvation, completion overhead, latency,
  cancellation time, loss, restart behavior, and CPU;
- retain the field geometry unless an alternative is at least as robust across
  the complete matrix.

The USB standard guarantees bulk packet semantics, not one universal optimal
userspace request size. The field-proven baseline remains the compatibility
default until evidence replaces it.

## Possible Rust/nusb stage

Only after C++ parity, tightening, and dogfooding:

- export the identical C ABI;
- reproduce the proven state machine and fail-fast boundaries;
- retain separate USB/consumer buffer capacity;
- keep a guaranteed pending endpoint depth;
- explicitly cancel and drain;
- preserve ordered retirement and stream epochs;
- reproduce controls, samples, drops, and callback behavior through common
  fixtures.

Rust ownership is used to make lifetime violations harder. It is not treated
as evidence of better USB scheduling or throughput.

## Explicit non-goals for the next advancement

- no transfer-size tuning;
- no switch from libusb to nusb;
- no public API/ABI change;
- no new framing or sequence header;
- no broad “recover every error” policy;
- no change from dropping the newest completed host block when the consumer
  queue is full;
- no SIMD or fused DSP before scalar parity;
- no lock-free queue without measured cross-platform benefit;
- no assumption that USB completion boundaries equal firmware bank boundaries.

## Implementation order

1. Freeze exported ABI and exact control/sample fixtures.
2. Introduce the C++ C-boundary and typed device/control objects.
3. Reproduce the sixteen-transfer plus eight-replacement pool.
4. Reproduce the USB-worker/consumer split and immediate resubmission.
5. Reproduce conversion and callback output bit-for-bit.
6. Add explicit stream generations and atomic lifecycle state.
7. Make start/stop/cancel transactional and fully retired.
8. Add classified internal diagnostics without broadening recovery.
9. Dogfood against stock and universal firmware on R2 and Mini.
10. Run the Windows/macOS/Linux compatibility matrix.
11. Only then open a separate transfer-tuning work item.

## Exit criteria

- Exported ABI and control traces match the C baseline.
- Scalar callback output is bit-identical for accepted golden streams.
- Sixteen USB transfers remain pending independently of consumer ownership.
- Slow callbacks cause the same counted host-buffer-drop policy without
  starving USB submission.
- Every submitted request reaches an observed terminal completion before free.
- Stop, callback-stop, failed start, unplug, stall, and restart have one defined
  lifecycle outcome with no use-after-free or self-join.
- Full USB buffers containing firmware discontinuities continue.
- Ambiguous short/epoch/order failures terminate cleanly with evidence.
- Stock firmware, universal firmware, R2, and Mini all pass.
- No tuning or Rust claim is mixed into the parity/tightening review.

