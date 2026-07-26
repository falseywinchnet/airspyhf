# Instructions: USB endpoint flush correctness, and an optional flash lock

Two work items. Item 1 is a correctness fix against the documented controller
procedure and should be taken regardless of threat model. Item 2 is a build
toggle that ships disabled.

Item 3 in the original list — enabling `SEE` and handling `SEI` — is **already
implemented** and needs no work. `common/usb.c:677` enables
`USB0_USBINTR_D_SEE`, and `airspy_m0/airspy_m0.c:243` handles `USB0_USBSTS_D_SEI`
by counting it and calling `cpu_reset()`. That is the correct response: the
controller halts itself after an AHB-master fault and only full
re-initialisation is valid. Leave it alone.

---

## Item 1 — `ENDPTSTAT` re-check in the endpoint flush

**File:** `device/common/usb.c`, `usb_flush_primed_endpoints()` at line 231.

### The defect

The current sequence is:

```c
usb_wait_for_endpoint_priming_to_finish(mask);   /* wait ENDPTPRIME clear */
usb_flush_endpoints(mask);                       /* write ENDPTFLUSH */
usb_wait_for_endpoint_flushing_to_finish(mask);  /* wait ENDPTFLUSH clear */
```

This does not establish that the endpoint is quiescent, and UM10503 says so
directly.

§25.6.20, ENDPTFLUSH:

> Writing a one to a bit(s) in this register will cause the associated
> endpoint(s) to clear any primed buffers. **If a packet is in progress for one
> of the associated endpoints, then that transfer will continue until
> completion.** Hardware will clear this register after the endpoint flush
> operation is successful.

§25.6.21, ENDPTSTAT:

> Buffer ready is cleared by USB reset, by the USB DMA system, or through the
> ENDPTFLUSH register.
>
> **Remark:** These bits will be momentarily cleared by hardware during hardware
> endpoint re-priming operations when a dTD is retired and the dQH is updated.

So a packet already in flight completes *through* the flush, retires its dTD,
and re-primes from the dQH. `ENDPTFLUSH` reading clear proves the flush command
was accepted, not that the controller has stopped touching the queue head.

The caller, `usb_endpoint_configure()` at line 704, immediately overwrites the
live dQH — `capabilities`, `current_dtd_pointer`, `next_dtd_pointer`,
`total_bytes`, and all five `buffer_pointer_page` entries. If the controller is
mid-retire it can latch a partially written queue head.

UM10503 §25.9.1 names this condition and names the trigger:

> In USB device control case, if the system error occurs, check if the endpoint
> transfer descriptor are programmed perfectly by software application during
> set_configuration request. The system error will occur if the hardware
> accesses inaccessible memory space for end point access.

Our `SET_CONFIGURATION` path reaches this on every request, including a repeat
with the current configuration value: the guard in
`usb_standard_request.c:113` wraps only the assignment, so
`usb_configuration_changed_cb()` runs unconditionally, and
`airspy_m0.c:454` calls `usb_endpoint_init()` on the bulk IN endpoint while it
is streaming.

### The fix

Loop the flush until `ENDPTSTAT` shows the endpoint is no longer ready. This is
the standard procedure for this controller family:

```c
static void usb_flush_primed_endpoints(const uint32_t mask)
{
  /*
   * UM10503 25.6.20: a packet already in progress continues through the flush,
   * and 25.6.21 notes the status bits are momentarily cleared while hardware
   * re-primes on dTD retirement. ENDPTFLUSH going clear therefore does not
   * prove the endpoint is quiescent. Repeat until ENDPTSTAT agrees, because the
   * caller overwrites the live dQH as soon as this returns.
   */
  do {
    usb_wait_for_endpoint_priming_to_finish(mask);
    usb_flush_endpoints(mask);
    usb_wait_for_endpoint_flushing_to_finish(mask);
  } while (USB0_ENDPTSTAT & mask);
}
```

Bound the outer loop. Use the existing `usb_wait_deadline_expired()` style so a
wedged controller cannot spin M0 forever.

### Fail closed on deadline expiry

`usb_wait_for_endpoint_priming_to_finish()` (line 196) and
`usb_wait_for_endpoint_flushing_to_finish()` (line 217) currently give up on
deadline expiry and return, after which the caller proceeds to rewrite the dQH
anyway. That is the unsafe branch, and it becomes *more* likely under load —
exactly when it is least safe.

Change these to report failure to the caller, and have `usb_endpoint_configure()`
skip the dQH rewrite when the flush could not be proven complete. Increment a
counter in the stream contract for the skipped case. Do not rewrite a queue head
the hardware may still own.

### What not to change

Leave `SET_CONFIGURATION` semantics alone. USB 2.0 §9.4.7 requires a device to
accept `SET_CONFIGURATION` with its current value and reset endpoint state, so
making the handler idempotent would be non-conformant. The fix belongs in making
the teardown safe, not in refusing or short-circuiting the request.

---

## Item 2 — `PREVENT_FLASH` build toggle

**Default: off.** Stock behaviour is unchanged unless the builder asks for it.

### Rationale

`AIRSPY_SPIFLASH_ERASE`, `AIRSPY_SPIFLASH_WRITE`, and
`AIRSPY_SPIFLASH_ERASE_SECTOR` are unauthenticated vendor requests. Any process
that can open the device can rewrite or erase the firmware image. Bounds
checking against flash size is present and correct, so this is not a memory
safety issue — overwriting the image is the command's purpose. The exposure is
that the capability is available to anyone holding the handle.

This is a deployment decision, not a defect, so it is a build option rather than
a behaviour change. Sites that want the device defended from remote reflash
accept that reflashing then requires physical access.

### Implementation

Add a make variable `PREVENT_FLASH`, defaulting to off, in the same place
`RELEASE` is handled (`device/common/Makefile_inc.mk:51`). When set, define
`AIRSPY_PREVENT_FLASH` for the M0 build.

In `airspy_m0/airspy_usb_req.c`, guard registration of the three mutating
handlers in `airspy_usb_req_init()`:

```c
#ifndef AIRSPY_PREVENT_FLASH
  vendor_request_handler[AIRSPY_SPIFLASH_ERASE] = usb_vendor_request_erase_spiflash;
  vendor_request_handler[AIRSPY_SPIFLASH_WRITE] = usb_vendor_request_write_spiflash;
  vendor_request_handler[AIRSPY_SPIFLASH_ERASE_SECTOR] = usb_vendor_request_erase_sector_spiflash;
#endif
```

Leave the entries unregistered rather than adding a runtime check. An
unregistered handler already returns `USB_REQUEST_STATUS_STALL` through the
existing dispatch in `usb_vendor_request()`, the host sees a clean stall, and no
code runs on the request path. Prefer `#ifndef` around the registrations over
`#ifdef` inside each handler so the handler bodies are also dropped by
`--gc-sections`.

**Keep `AIRSPY_SPIFLASH_READ` registered.** It does not mutate anything, and
removing it would break existing diagnostics for no defensive gain.

Nothing on the streaming path changes, and nothing is added to any hot loop.

### Recovery when enabled

With `PREVENT_FLASH=on` the device cannot be reflashed over USB. Recovery is via
the LPC43xx USB0 boot ROM, which requires the ISP pin state to be forced — that
is, opening the enclosure. Recovery images are in `firmware/hardware/recovery/`.

### Documentation

State plainly in `BUILDING.md`:

- the toggle exists, defaults to off, and what it disables;
- that a device built with it on cannot be field-updated over USB;
- that recovery requires physical access and a known-good image.

A builder must not discover this after shipping units.
