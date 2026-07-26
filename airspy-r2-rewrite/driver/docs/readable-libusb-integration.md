# Readable libusb integration

Status: hardware-capable development candidate  
SDR# Wine helper: readable candidate installed  
Transfer geometry: unchanged

The ownership model is now connected to real libusb transfers in `readable/`.
This is no longer only a simulator: the candidate has streamed from both the
Airspy Mini and Airspy R2 attached to the development Mac.

## What is implemented

The candidate retains the field geometry:

```text
16 submitted bulk-IN transfers x 256 KiB
8 consumer/replacement buffers x 256 KiB
24 physical allocations, moved by pointer ownership
```

Each submitted transfer has an explicit slot and generation. A completion must
retire that exact slot before any delivery, resubmission, cancellation, or
error policy runs. The physical data buffers have a separate owner model, so a
consumer-held block cannot also be used as an active USB destination.

For a full completion the order is:

1. classify and retire the completed request;
2. exchange its physical buffer with a consumer replacement under the queue
   mutex;
3. release the queue mutex;
4. resubmit the same libusb request with the replacement buffer;
5. signal the consumer only after endpoint depth has been restored.

If the consumer queue is full, the completed host block is deliberately
dropped, the drop is accumulated, and the request is immediately resubmitted
without transferring ownership. The next delivered block carries the pending
count through the existing `dropped_samples` ABI.

## Start and stop

New firmware starts as:

```text
OFF -> clear endpoint halt -> ARMED -> submit requests/start workers -> RX
```

If `ARMED` is rejected, the candidate preserves the old-firmware fallback:

```text
OFF -> clear endpoint halt -> RX -> submit requests/start workers
```

Submission is transactional. If request N cannot be submitted, the already
submitted prefix is cancelled and the caller pumps libusb until every one has
produced a terminal callback. Storage is never freed while the request ledger
has a pending member.

Normal stop changes the receiver to `OFF`, requests cancellation, keeps the
event thread alive until every active request retires, joins both workers, and
only then permits buffer or request destruction.

Calling `airspy_stop_rx()` from the application callback is explicitly
supported. That call requests asynchronous shutdown and returns without joining
its own consumer thread. A later external stop or close performs final joining
and retirement.

## Errors

The internal completion classifier distinguishes:

- full completion;
- short completion;
- cancellation;
- endpoint stall;
- disconnect;
- overflow;
- generic backend failure.

Only a full, current-generation completion is eligible for delivery and
resubmission. Short or otherwise ambiguous transport completion remains
fail-fast. This preserves the conservative policy while removing untracked
request lifetime.

## Hardware and sanitizer evidence

The bounded smoke test configures real unsigned-16 samples at samplerate index
zero and runs these cases on every enumerated R2/Mini:

- callback returns stop after 32 blocks;
- callback directly calls `airspy_stop_rx()` after 32 blocks;
- callback sleeps 100 ms for 12 blocks, forcing counted host-queue drops;
- packing is enabled, 32 blocks are received, and packing is disabled again;
- device close after all transitions.

On 2026-07-25 both serials passed:

```text
Mini  35AC63DC2D6ABB4F
R2    35AC63DC2D7D704F
```

The same matrix passed:

- the ordinary debug candidate;
- AddressSanitizer plus UndefinedBehaviorSanitizer;
- ThreadSanitizer.

The slow-consumer case produced nonzero `dropped_samples` on both radios while
streaming continued, demonstrating that USB requests remain reusable when the
consumer is intentionally slower than the source.

## Bugs found while connecting the model

Hardware testing exposed two stop-path races in the first integration:

1. two external finalizers could concurrently join or alter worker state;
2. an external finalizer could hold lifecycle serialization while joining the
   consumer as that consumer callback tried to enter the same serialized
   `airspy_stop_rx()` path.

Lifecycle finalization is now serialized. Callback-side stop is detected before
that serialization and is limited to the nonjoining stop request. Worker
presence is represented with atomics rather than the legacy cross-thread
booleans.

## SDR# helper dogfood

The Wine bridge now builds the release readable driver itself, links the native
helper to that artifact by explicit pathname, copies it beside the helper, and
rewrites the helper load command to:

```text
@loader_path/libairspy.0.dylib
```

The helper logs the resolved dylib pathname at startup. The live process was
also inspected to confirm that exact adjacent file was mapped. The Windows
shim installed beside the launcher's `SDRSharp.exe` was checksum-identical to
the newly built shim.

An end-to-end Wine probe through the shim and helper enumerated both radios,
opened the Mini, applied controls, and delivered 6.029 MS/s real for one second
with zero host drops before stopping cleanly.

## What this does not prove yet

- Actual partial-submit failure has model coverage but no injected libusb
  hardware test.
- Disconnect, endpoint stall, and genuine short-completion hardware injection
  still need lifecycle tests.
- General concurrent control calls are not yet a supported contract; the
  helper serializes controls.
- A backend that never returns terminal callbacks is intentionally allowed to
  keep stop waiting rather than freeing active storage. A bounded user-visible
  policy for that pathological backend state remains to be designed.
- Windows and Linux have not passed the compatibility matrix.
- The helper dogfood run is bounded; sustained SDR# operation remains the next
  field test.

The result is installed for SDR# dogfooding, not yet declared a general
replacement for the field driver.
