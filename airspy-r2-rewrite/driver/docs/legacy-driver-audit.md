# Legacy libairspy driver audit

Status: first refactor audit, 2026-07-25  
Scope: `current/libairspy/src/airspy.c`  
Policy: preserve observable compatibility; do not preserve undefined lifetime
or arithmetic behavior

This is not an argument that the legacy driver was generally unsuccessful. Its
steady-state transport shape is sound: sixteen asynchronous requests, eight
replacement buffers, a small completion callback, and a separate consumer.
This audit identifies places where that design is represented by rules the C
program does not actually enforce.

Line references below identify the current parity oracle. The readable C++
implementation must carry each invariant explicitly and prove it with a model
or failure-injection test before it replaces the C implementation.

## A. Active libusb requests can be freed before terminal completion

Original stop sequence:

```c
device->streaming = false;
cancel_transfers(device);
pthread_join(device->transfer_thread, NULL);
libusb_handle_events_timeout_completed(device->usb_context, &timeout, NULL);
```

The transfer thread exits as soon as `streaming` becomes false. Cancellation is
asynchronous: `libusb_cancel_transfer()` requests cancellation, but each
submitted request remains active until its callback reports a terminal
completion. One zero-time event call does not prove that all sixteen callbacks
have happened. `airspy_close()` subsequently calls `free_transfers()`, which
frees every request and data buffer unconditionally.

Libusb explicitly makes freeing an active transfer undefined behavior. On
Darwin, cancelling one request also cancels every request on that endpoint, so
this is particularly important for the Mac path.

Disposition:

- count each successful submission;
- observe exactly one terminal callback for every submission;
- continue event handling during drain;
- free request and buffer storage only when the pending count is zero.

Model: `TransferLedger::can_release_storage()`.

## B. Partial pool submission has no rollback

Original pool submission:

```c
for (...) {
    error = libusb_submit_transfer(device->transfers[transfer_index]);
    if (error != 0) {
        return AIRSPY_ERROR_LIBUSB;
    }
}
```

If request `N` fails, requests `0..N-1` remain submitted. `create_io_threads()`
returns before starting the event thread, leaves `streaming` true, and provides
no cancellation/drain path. A retry reports busy; close reaches the unsafe free
path described above.

Disposition:

- submission is transactional;
- a partial pool is cancelled and fully drained;
- the start generation cannot become streaming;
- failure injection covers every possible failed submission index.

## C. Partial thread creation has no rollback

The consumer thread is created first. If creation of the transfer/event thread
fails, `create_io_threads()` returns immediately while:

- the USB pool is submitted;
- the consumer thread is alive or waiting;
- `streaming` remains true;
- the thread attributes are not destroyed;
- no owner performs a complete unwind.

Disposition:

- threads and submissions are scoped start resources;
- every failed construction step unwinds prior steps in reverse order;
- a failed start returns to `open` only after all workers and requests retire.

## D. Callback-initiated stop can join the calling thread

`airspy_stop_rx()` calls `kill_io_threads()`, which joins both worker threads.
If an application calls `airspy_stop_rx()` from its sample callback, the
consumer thread attempts to join itself. Depending on the pthread
implementation this deadlocks or returns an error that the driver ignores.
The driver then clears `consumer_thread_running` even though the worker may
still be executing.

Disposition:

- callback stop only requests asynchronous stop;
- the consumer never joins itself;
- an external owner or the event worker performs drain/finalization;
- callback return and explicit callback-side stop converge on one state path.

## E. `volatile` is being used as cross-thread synchronization

`streaming` and `stop_requested` are read and written by the caller, event
thread, consumer thread, and libusb callback without a common mutex or atomic
operations. `volatile` prevents some compiler elision; it does not establish
inter-thread ordering or eliminate a C data race.

Consequences are not limited to stale UI state. These flags guard:

- whether a transfer callback resubmits;
- whether transfer storage may be reconfigured;
- whether worker loops continue;
- whether filter objects can be replaced.

Disposition:

- one explicit atomic lifecycle state and monotonically increasing generation;
- mutex protection for compound queue state;
- acquire/release edges where ownership crosses threads;
- no independent booleans capable of expressing contradictory states.

## F. A completion can silently cease being tracked

The libusb callback immediately returns when `streaming` is false or
`stop_requested` is true:

```c
if (!device->streaming || device->stop_requested) {
    return;
}
```

That is acceptable only if another mechanism records the request's terminal
completion. None does. The callback does not distinguish completed, cancelled,
stalled, disconnected, or overflowed requests during shutdown and does not
decrement a submitted count.

Disposition:

- every callback retires ownership first, regardless of stream policy;
- only then decide whether a valid full completion should be delivered and
  resubmitted;
- terminal reason is recorded separately from application-visible behavior.

## G. Signed left shift invokes undefined behavior for half the ADC range

Original INT16 conversion:

```c
dest[i] = (src[i] - 2048) << 4;
```

For input values below 2048, the left operand is negative. Left-shifting a
negative signed integer is undefined behavior in C and C++. The intended
operation is exactly representable as multiplication by 16.

Disposition:

```text
centered = int(sample & 0x0fff) - 2048
output = int16_t(centered * 16)
```

The readable scalar conversion implements this and tests both endpoints,
midpoint, and neighboring values.

## H. Invalid sample types can expose an uninitialized sample pointer

`airspy_set_sample_type()` accepts every integer representable by the enum. In
the consumer switch, `AIRSPY_SAMPLE_END` deliberately does not assign
`transfer.samples`; the callback is then invoked with that indeterminate
pointer. Out-of-range values do not match any case and have the same result.

Disposition:

- validate `0 <= sample_type < AIRSPY_SAMPLE_END`;
- reject changes while a generation is streaming unless legacy behavior
  explicitly requires them;
- construct each callback record from initialized fields.

## I. Filter replacement destroys the working filter before allocation succeeds

Both conversion-filter setters free the current converter, call a constructor,
ignore a null result, and return success. The next start resets or processes
through the null converter.

Disposition:

- construct the replacement first;
- return `AIRSPY_ERROR_NO_MEM` on failure;
- swap only after successful construction;
- destroy the old converter after the swap.

## J. Pool allocation is non-transactional

Every allocation failure returns immediately without releasing objects already
created in that attempt. The transfer-pointer array is also allocated with
`sizeof(struct libusb_transfer)` instead of `sizeof(pointer)`. That particular
mistake overallocates on current systems rather than corrupting memory, but it
is evidence that allocation ownership is implicit. Clang's static analyzer
independently reports the mismatched `calloc` element type.

Disposition:

- construct into an isolated owner;
- release partial allocation automatically;
- publish the new pool only when all 24 buffers, sixteen requests, conversion
  storage, and optional unpack storage exist.

## K. Packing reconfiguration trusts a racy idle observation

`airspy_set_packing()` tests the unsynchronized `streaming` flag, calls
`cancel_transfers()`, and immediately frees the entire pool without observing
terminal callbacks. Ordinarily the API is called while stopped, but a
concurrent error or control call can make `streaming` false while requests
remain active.

Disposition:

- only the `open` lifecycle state may replace a pool;
- `open` implies pending transfer count zero and no worker ownership;
- packing changes are transactional and leave the prior pool usable on failure.

## L. Enumeration error paths and arguments are brittle

Examples:

- a negative `count` with non-null `serials` becomes a very large `memset`;
- failure of `libusb_get_device_list()` returns without `libusb_exit()`;
- several public functions dereference device/output pointers without
  validation;
- serial parsing accepts a prefix conversion without proving the complete
  expected hexadecimal field was consumed.

These are not hot-path concerns, but the C ABI boundary is exactly where inputs
must be bounded before entering the C++ core.

## M. The installed header promises two symbols the library does not define

`airspy.h` declares `airspy_config_write()` and `airspy_config_read()`, but the
1.0.12 library contains neither implementation nor exported symbol. A program
can compile against the installed public header and then fail to link.

Conversely, the binary exports `airspy_set_receiver_mode()` and six gain-table
objects that are not all part of the intended public surface. The rewrite must
distinguish documented API from actual field ABI before choosing whether to
add, retain, or hide anything.

Disposition:

- preserve every symbol present in `docs/legacy-exported-symbols.txt`;
- implement the two header-promised config functions or explicitly correct the
  installed header as a separately reviewed compatibility decision;
- compare candidate exports automatically on every platform build.

## What the first readable extraction now proves

The model currently makes these rules executable:

- exactly sixteen USB request slots;
- a generation-qualified submitted/cancelled/completed lifecycle;
- storage cannot be released while any request remains pending;
- prepared start submits the pool before `RX`;
- legacy fallback retains its one-step order but rolls back failed submission;
- failed prepared start stops the receiver and drains the pool;
- sixteen USB buffers and eight consumer/replacement buffers remain distinct;
- a full consumer queue drops a completed host block without starving the USB
  transfer of a replacement buffer;
- pending host-drop accounting attaches to the next delivered block;
- all 24 data-buffer identities remain unique through swaps;
- signed-safe scalar U12-to-I16 conversion matches intended legacy output.

Those owners are now connected to actual libusb callbacks in the `readable/`
candidate. Both local radios pass normal, callback-stop, slow-consumer, packed,
ASan/UBSan, and TSan smoke runs. The exact scope and remaining gates are recorded
in `docs/readable-libusb-integration.md`; the candidate has not yet replaced the
production helper.
