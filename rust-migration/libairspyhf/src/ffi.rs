/*
Copyright (c) 2016-2023, Youssef Touil <youssef@airspy.com>
Copyright (C) 2024, Joshuah Rainstar <joshuah.rainstar@gmail.com>

The exported C ABI, matching `airspyhf.h`. Each function is a thin wrapper over
the safe methods on `AirspyHfDevice` in `device.rs`, and every entry point is
wrapped in a panic guard so a Rust panic can never unwind across the `extern "C"`
boundary (which is undefined / aborts the host).
*/

//! C ABI surface (mirrors `airspyhf.h`).

use std::ffi::c_void;
use std::panic::{catch_unwind, AssertUnwindSafe};

use crate::commands::{
    AIRSPYHF_GET_BIAS_TEE_COUNT, AIRSPYHF_GET_BIAS_TEE_NAME, AIRSPYHF_GET_SERIALNO_BOARDID,
    AIRSPYHF_GET_VERSION_STRING, MAX_VERSION_STRING_SIZE,
};
use crate::device::AirspyHfDevice;
use crate::{
    AirspyhfLibVersion, AirspyhfReadPartIdSerialNo, AirspyhfSampleBlockCbFn, AIRSPYHF_VER_MAJOR,
    AIRSPYHF_VER_MINOR, AIRSPYHF_VER_REVISION, SAMPLES_TO_TRANSFER,
};

/// FFI-safe pointer type used by the C interface.
pub type AirspyhfDeviceHandle = *mut AirspyHfDevice;

/// Error codes mirroring `enum airspyhf_error`.
#[repr(C)]
pub enum AirspyhfError {
    Success = 0,
    Error = -1,
    Unsupported = -2,
}

/// Run an FFI body under a panic guard, returning `default` if it unwinds.
///
/// This is the single point that upholds the "no unwinding across `extern "C"`"
/// rule for the whole ABI. `AssertUnwindSafe` is sound here because on a panic we
/// discard all state and return an error code; we never observe a
/// logically-broken value afterwards.
fn ffi_guard<T>(default: T, f: impl FnOnce() -> T) -> T {
    catch_unwind(AssertUnwindSafe(f)).unwrap_or(default)
}

const OK: i32 = AirspyhfError::Success as i32;
const ERR: i32 = AirspyhfError::Error as i32;

/// Map an `io::Result<()>` to the C success/error code.
fn code(res: std::io::Result<()>) -> i32 {
    match res {
        Ok(()) => OK,
        Err(_) => ERR,
    }
}

// ---- Open / close --------------------------------------------------------

/// # Safety
/// `out` must be a valid, writable pointer.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_open(out: *mut AirspyhfDeviceHandle) -> i32 {
    ffi_guard(ERR, || {
        if out.is_null() {
            return ERR;
        }
        // SAFETY: caller guarantees `out` is writable.
        unsafe { *out = std::ptr::null_mut() };
        match AirspyHfDevice::open_first() {
            Ok(d) => {
                // SAFETY: caller guarantees `out` is writable.
                unsafe { *out = Box::into_raw(Box::new(d)) };
                OK
            }
            Err(_) => ERR,
        }
    })
}

/// # Safety
/// `out` must be a valid, writable pointer.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_open_sn(out: *mut AirspyhfDeviceHandle, serial: u64) -> i32 {
    ffi_guard(ERR, || {
        if out.is_null() {
            return ERR;
        }
        // SAFETY: caller guarantees `out` is writable.
        unsafe { *out = std::ptr::null_mut() };
        match AirspyHfDevice::open_by_serial(serial) {
            Ok(d) => {
                // SAFETY: caller guarantees `out` is writable.
                unsafe { *out = Box::into_raw(Box::new(d)) };
                OK
            }
            Err(_) => ERR,
        }
    })
}

/// # Safety
/// `out` must be a valid, writable pointer. `fd` must be an owned USB file
/// descriptor (Linux/Android only).
#[no_mangle]
pub unsafe extern "C" fn airspyhf_open_fd(out: *mut AirspyhfDeviceHandle, fd: i32) -> i32 {
    #[cfg(any(target_os = "linux", target_os = "android"))]
    {
        ffi_guard(ERR, || {
            if out.is_null() {
                return ERR;
            }
            // SAFETY: caller guarantees `out` is writable.
            unsafe { *out = std::ptr::null_mut() };
            use std::os::fd::{FromRawFd, OwnedFd};
            // SAFETY: caller passes ownership of a valid USB fd.
            let owned = unsafe { OwnedFd::from_raw_fd(fd) };
            match AirspyHfDevice::open_from_fd(owned) {
                Ok(d) => {
                    // SAFETY: caller guarantees `out` is writable.
                    unsafe { *out = Box::into_raw(Box::new(d)) };
                    OK
                }
                Err(_) => ERR,
            }
        })
    }
    #[cfg(not(any(target_os = "linux", target_os = "android")))]
    {
        let _ = (out, fd);
        AirspyhfError::Unsupported as i32
    }
}

/// # Safety
/// `dev` must have come from an `airspyhf_open*` call and is freed here.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_close(dev: AirspyhfDeviceHandle) -> i32 {
    ffi_guard(ERR, || {
        if dev.is_null() {
            return ERR;
        }
        // SAFETY: `dev` came from Box::into_raw in an open fn and is not aliased.
        // Drop runs stop_streaming; the balancer frees via RAII.
        drop(unsafe { Box::from_raw(dev) });
        OK
    })
}

/// # Safety
/// `serials` may be null; if non-null it must hold at least `count` u64 slots.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_list_devices(serials: *mut u64, count: i32) -> i32 {
    ffi_guard(ERR, || match AirspyHfDevice::list_serials() {
        Ok(list) => {
            if !serials.is_null() && count > 0 {
                let len = list.len().min(count as usize);
                // SAFETY: caller guarantees `serials` holds `count` slots.
                unsafe { std::ptr::copy_nonoverlapping(list.as_ptr(), serials, len) };
            }
            list.len() as i32
        }
        Err(_) => ERR,
    })
}

/// # Safety
/// `ver` must be a valid, writable pointer.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_lib_version(ver: *mut AirspyhfLibVersion) {
    ffi_guard((), || {
        // SAFETY: caller guarantees `ver` is writable (or null, checked here).
        if let Some(v) = unsafe { ver.as_mut() } {
            v.major_version = AIRSPYHF_VER_MAJOR;
            v.minor_version = AIRSPYHF_VER_MINOR;
            v.revision = AIRSPYHF_VER_REVISION;
        }
    })
}

// ---- Status --------------------------------------------------------------

/// # Safety
/// `dev` must be a valid device handle or null.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_get_output_size(_dev: AirspyhfDeviceHandle) -> i32 {
    SAMPLES_TO_TRANSFER
}

/// # Safety
/// `dev` must be a valid device handle or null.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_is_low_if(dev: AirspyhfDeviceHandle) -> i32 {
    ffi_guard(0, || {
        // SAFETY: caller guarantees `dev` is a valid handle or null.
        match unsafe { dev.as_ref() } {
            Some(d) if d.is_low_if() => 1,
            _ => 0,
        }
    })
}

/// # Safety
/// `dev` must be a valid device handle or null.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_is_streaming(dev: AirspyhfDeviceHandle) -> i32 {
    ffi_guard(0, || {
        // SAFETY: caller guarantees `dev` is a valid handle or null.
        match unsafe { dev.as_ref() } {
            Some(d) if d.is_streaming() => 1,
            _ => 0,
        }
    })
}

// ---- Streaming -----------------------------------------------------------

/// # Safety
/// `dev` must be a valid device handle; `ctx` is passed back to `cb` unchanged.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_start(
    dev: AirspyhfDeviceHandle,
    cb: AirspyhfSampleBlockCbFn,
    ctx: *mut c_void,
) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees exclusive access to a valid `dev`.
        let Some(d) = (unsafe { dev.as_mut() }) else {
            return ERR;
        };
        code(d.start_streaming(cb, ctx))
    })
}

/// # Safety
/// `dev` must be a valid device handle.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_stop(dev: AirspyhfDeviceHandle) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees exclusive access to a valid `dev`.
        let Some(d) = (unsafe { dev.as_mut() }) else {
            return ERR;
        };
        d.stop_streaming();
        OK
    })
}

// ---- DSP toggles ---------------------------------------------------------

/// # Safety
/// `dev` must be a valid device handle.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_lib_dsp(dev: AirspyhfDeviceHandle, flag: u8) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees exclusive access to a valid `dev`.
        let Some(d) = (unsafe { dev.as_mut() }) else {
            return ERR;
        };
        d.set_enable_dsp(flag);
        OK
    })
}

/// # Safety
/// `dev` must be a valid device handle.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_optimal_iq_correction_point(
    dev: AirspyhfDeviceHandle,
    w: f32,
) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees exclusive access to a valid `dev`.
        let Some(d) = (unsafe { dev.as_mut() }) else {
            return ERR;
        };
        d.set_iq_correction_point(w);
        OK
    })
}

/// # Safety
/// `dev` must be a valid device handle.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_iq_balancer_configure(
    dev: AirspyhfDeviceHandle,
    b: i32,
    fi: i32,
    fo: i32,
    ci: i32,
) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees exclusive access to a valid `dev`.
        let Some(d) = (unsafe { dev.as_mut() }) else {
            return ERR;
        };
        d.iq_balancer_configure(b, fi, fo, ci);
        OK
    })
}

// ---- Sample rate / frequency ---------------------------------------------

/// # Safety
/// `dev` must be a valid device handle. `buffer` must hold `len` u32s (or be the
/// count out-pointer when `len == 0`).
#[no_mangle]
pub unsafe extern "C" fn airspyhf_get_samplerates(
    dev: AirspyhfDeviceHandle,
    buffer: *mut u32,
    len: u32,
) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees `dev` is valid.
        let Some(d) = (unsafe { dev.as_ref() }) else {
            return ERR;
        };
        let rates = d.supported_samplerates();
        if len == 0 {
            if !buffer.is_null() {
                // SAFETY: `buffer` is the count out-pointer.
                unsafe { *buffer = rates.len() as u32 };
            }
        } else if (len as usize) <= rates.len() {
            if !buffer.is_null() {
                // SAFETY: caller guarantees `buffer` holds `len` u32s.
                unsafe { std::ptr::copy_nonoverlapping(rates.as_ptr(), buffer, len as usize) };
            }
        } else {
            return ERR;
        }
        OK
    })
}

/// # Safety
/// `dev` must be a valid device handle.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_samplerate(
    dev: AirspyhfDeviceHandle,
    samplerate: u32,
) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees exclusive access to a valid `dev`.
        let Some(d) = (unsafe { dev.as_mut() }) else {
            return ERR;
        };
        code(d.set_samplerate(samplerate))
    })
}

/// # Safety
/// `dev` must be a valid device handle.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_freq(dev: AirspyhfDeviceHandle, freq_hz: u32) -> i32 {
    // SAFETY: forwards to the checked f64 variant.
    unsafe { airspyhf_set_freq_double(dev, freq_hz as f64) }
}

/// # Safety
/// `dev` must be a valid device handle.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_freq_double(dev: AirspyhfDeviceHandle, freq_hz: f64) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees exclusive access to a valid `dev`.
        let Some(d) = (unsafe { dev.as_mut() }) else {
            return ERR;
        };
        code(d.set_freq(freq_hz))
    })
}

// ---- Frontend / calibration ----------------------------------------------

/// # Safety
/// `dev` and `flags` must be valid pointers.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_get_frontend_options(
    dev: AirspyhfDeviceHandle,
    flags: *mut u32,
) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees `dev`/`flags` are valid or null.
        match (unsafe { dev.as_ref() }, unsafe { flags.as_mut() }) {
            (Some(d), Some(f)) => {
                *f = d.frontend_options();
                OK
            }
            _ => ERR,
        }
    })
}

/// # Safety
/// `dev` must be a valid device handle.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_frontend_options(
    dev: AirspyhfDeviceHandle,
    flags: u32,
) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees exclusive access to a valid `dev`.
        let Some(d) = (unsafe { dev.as_mut() }) else {
            return ERR;
        };
        code(d.set_frontend_options(flags))
    })
}

/// # Safety
/// `dev` and `ppb` must be valid pointers.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_get_calibration(dev: AirspyhfDeviceHandle, ppb: *mut i32) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees `dev`/`ppb` are valid or null.
        match (unsafe { dev.as_ref() }, unsafe { ppb.as_mut() }) {
            (Some(d), Some(p)) => {
                *p = d.calibration_ppb();
                OK
            }
            _ => ERR,
        }
    })
}

/// # Safety
/// `dev` must be a valid device handle.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_calibration(dev: AirspyhfDeviceHandle, ppb: i32) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees exclusive access to a valid `dev`.
        let Some(d) = (unsafe { dev.as_mut() }) else {
            return ERR;
        };
        code(d.set_calibration(ppb))
    })
}

/// # Safety
/// `dev` and `vc` must be valid pointers.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_get_vctcxo_calibration(
    dev: AirspyhfDeviceHandle,
    vc: *mut u16,
) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees `dev`/`vc` are valid or null.
        match (unsafe { dev.as_ref() }, unsafe { vc.as_mut() }) {
            (Some(d), Some(v)) => {
                *v = d.calibration_vctcxo();
                OK
            }
            _ => ERR,
        }
    })
}

/// # Safety
/// `dev` must be a valid device handle.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_vctcxo_calibration(
    dev: AirspyhfDeviceHandle,
    vc: u16,
) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees exclusive access to a valid `dev`.
        let Some(d) = (unsafe { dev.as_mut() }) else {
            return ERR;
        };
        code(d.set_vctcxo_calibration(vc))
    })
}

/// # Safety
/// `dev` must be a valid device handle; streaming must be stopped.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_flash_configuration(dev: AirspyhfDeviceHandle) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees `dev` is valid.
        let Some(d) = (unsafe { dev.as_ref() }) else {
            return ERR;
        };
        code(d.flash_configuration())
    })
}

// ---- Attenuation / bias tee / gain ---------------------------------------

/// # Safety
/// `dev` and `buffer` must be valid pointers; `buffer` holds `len` f32s (or the
/// count out-pointer when `len == 0`).
#[no_mangle]
pub unsafe extern "C" fn airspyhf_get_att_steps(
    dev: AirspyhfDeviceHandle,
    buffer: *mut c_void,
    len: u32,
) -> i32 {
    ffi_guard(ERR, || {
        if buffer.is_null() {
            return ERR;
        }
        // SAFETY: caller guarantees `dev` is valid.
        let Some(d) = (unsafe { dev.as_ref() }) else {
            return ERR;
        };
        let steps = d.supported_att_steps();
        if len == 0 {
            // SAFETY: `buffer` is the count out-pointer.
            unsafe { *(buffer as *mut u32) = steps.len() as u32 };
        } else if (len as usize) <= steps.len() {
            // SAFETY: caller guarantees `buffer` holds `len` f32s.
            unsafe {
                std::ptr::copy_nonoverlapping(steps.as_ptr(), buffer as *mut f32, len as usize)
            };
        } else {
            return ERR;
        }
        OK
    })
}

/// # Safety
/// `dev` must be a valid device handle.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_att(dev: AirspyhfDeviceHandle, att: f32) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees exclusive access to a valid `dev`.
        let Some(d) = (unsafe { dev.as_mut() }) else {
            return ERR;
        };
        code(d.set_att(att))
    })
}

/// # Safety
/// `dev` must be a valid device handle.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_bias_tee(dev: AirspyhfDeviceHandle, value: i8) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees exclusive access to a valid `dev`.
        let Some(d) = (unsafe { dev.as_mut() }) else {
            return ERR;
        };
        code(d.set_bias_tee(value))
    })
}

/// # Safety
/// `dev` and `count` must be valid pointers.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_get_bias_tee_count(
    dev: AirspyhfDeviceHandle,
    count: *mut i32,
) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees `dev`/`count` are valid or null.
        let (Some(d), Some(c)) = (unsafe { dev.as_ref() }, unsafe { count.as_mut() }) else {
            return ERR;
        };
        let mut buf = [0u8; 4];
        if d.vendor_in(AIRSPYHF_GET_BIAS_TEE_COUNT, 0, 0, &mut buf)
            .is_err()
        {
            return ERR;
        }
        *c = i32::from_le_bytes(buf);
        OK
    })
}

/// # Safety
/// `dev` must be valid; `name` must hold `length` bytes.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_get_bias_tee_name(
    dev: AirspyhfDeviceHandle,
    index: i32,
    name: *mut std::os::raw::c_char,
    length: u8,
) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees `dev` is valid.
        let Some(d) = (unsafe { dev.as_ref() }) else {
            return ERR;
        };
        // SAFETY: `name`/`length` describe the caller's buffer.
        unsafe { read_string(d, AIRSPYHF_GET_BIAS_TEE_NAME, 0, index as u16, name, length) }
    })
}

/// # Safety
/// `dev` and `out` must be valid pointers.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_board_partid_serialno_read(
    dev: AirspyhfDeviceHandle,
    out: *mut AirspyhfReadPartIdSerialNo,
) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees `dev` is valid.
        let Some(d) = (unsafe { dev.as_ref() }) else {
            return ERR;
        };
        if out.is_null() {
            return ERR;
        }
        let size = std::mem::size_of::<AirspyhfReadPartIdSerialNo>();
        let mut buf = [0u8; std::mem::size_of::<AirspyhfReadPartIdSerialNo>()];
        if d.vendor_in(AIRSPYHF_GET_SERIALNO_BOARDID, 0, 0, &mut buf)
            .is_err()
        {
            return ERR;
        }
        // SAFETY: `out` points to a struct of exactly `size` bytes.
        unsafe { std::ptr::copy_nonoverlapping(buf.as_ptr(), out as *mut u8, size) };
        OK
    })
}

/// # Safety
/// `dev` must be valid; `version` must hold `length` bytes.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_version_string_read(
    dev: AirspyhfDeviceHandle,
    version: *mut std::os::raw::c_char,
    length: u8,
) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees `dev` is valid.
        let Some(d) = (unsafe { dev.as_ref() }) else {
            return ERR;
        };
        // SAFETY: `version`/`length` describe the caller's buffer.
        unsafe { read_string(d, AIRSPYHF_GET_VERSION_STRING, 0, 0, version, length) }
    })
}

/// Shared helper for the null-terminated string vendor reads (version, bias-tee
/// name).
///
/// # Safety
/// `out` must be null or point to at least `length` writable bytes.
unsafe fn read_string(
    d: &AirspyHfDevice,
    request: u8,
    value: u16,
    index: u16,
    out: *mut std::os::raw::c_char,
    length: u8,
) -> i32 {
    if out.is_null() || length == 0 {
        return ERR;
    }
    let mut buf = [0u8; MAX_VERSION_STRING_SIZE];
    if d.vendor_in(request, value, index, &mut buf).is_err() {
        return ERR;
    }
    let copy_len = ((length as usize) - 1).min(buf.len());
    // SAFETY: caller guarantees `out` has room for `length` bytes; we write
    // `copy_len < length` bytes plus a NUL terminator.
    unsafe {
        std::ptr::copy_nonoverlapping(buf.as_ptr() as *const std::os::raw::c_char, out, copy_len);
        *out.add(copy_len) = 0;
    }
    OK
}

// ---- Legacy / HF controls ------------------------------------------------

/// # Safety
/// `dev` must be a valid device handle.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_user_output(
    dev: AirspyhfDeviceHandle,
    pin: u32,
    value: u32,
) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees exclusive access to a valid `dev`.
        let Some(d) = (unsafe { dev.as_mut() }) else {
            return ERR;
        };
        code(d.set_user_output(pin, value))
    })
}

/// # Safety
/// `dev` must be a valid device handle.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_hf_agc(dev: AirspyhfDeviceHandle, flag: u8) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees exclusive access to a valid `dev`.
        let Some(d) = (unsafe { dev.as_mut() }) else {
            return ERR;
        };
        code(d.set_hf_agc(flag))
    })
}

/// # Safety
/// `dev` must be a valid device handle.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_hf_agc_threshold(dev: AirspyhfDeviceHandle, flag: u8) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees exclusive access to a valid `dev`.
        let Some(d) = (unsafe { dev.as_mut() }) else {
            return ERR;
        };
        code(d.set_hf_agc_threshold(flag))
    })
}

/// # Safety
/// `dev` must be a valid device handle.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_hf_att(dev: AirspyhfDeviceHandle, index: u8) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees exclusive access to a valid `dev`.
        let Some(d) = (unsafe { dev.as_mut() }) else {
            return ERR;
        };
        code(d.set_hf_att(index))
    })
}

/// # Safety
/// `dev` must be a valid device handle.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_hf_lna(dev: AirspyhfDeviceHandle, flag: u8) -> i32 {
    ffi_guard(ERR, || {
        // SAFETY: caller guarantees exclusive access to a valid `dev`.
        let Some(d) = (unsafe { dev.as_mut() }) else {
            return ERR;
        };
        code(d.set_hf_lna(flag))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::AirspyhfTransfer;

    #[test]
    #[cfg_attr(miri, ignore)]
    fn open_close() {
        let mut handle: AirspyhfDeviceHandle = std::ptr::null_mut();
        unsafe {
            let _ = airspyhf_open(&mut handle as *mut _);
        }
        if !handle.is_null() {
            unsafe {
                assert_eq!(0, airspyhf_close(handle));
            }
        }
    }

    #[test]
    #[cfg_attr(miri, ignore)]
    fn list_devices_returns() {
        let count = unsafe { airspyhf_list_devices(std::ptr::null_mut(), 0) };
        assert!(count >= 0 || count == ERR);
    }

    #[test]
    #[cfg_attr(miri, ignore)]
    fn open_sn_invalid() {
        let mut handle: AirspyhfDeviceHandle = std::ptr::null_mut();
        let ret = unsafe { airspyhf_open_sn(&mut handle as *mut _, 0) };
        assert_ne!(ret, OK);
        assert!(handle.is_null());
    }

    // Null-handle calls must return errors, never panic or crash (panic guard).
    #[test]
    fn null_handle_is_safe() {
        let n: AirspyhfDeviceHandle = std::ptr::null_mut();
        unsafe {
            assert_eq!(ERR, airspyhf_open(std::ptr::null_mut()));
            assert_eq!(ERR, airspyhf_open_sn(std::ptr::null_mut(), 0));
            assert_eq!(ERR, airspyhf_close(n));
            assert_eq!(0, airspyhf_is_streaming(n));
            assert_eq!(0, airspyhf_is_low_if(n));
            assert_eq!(ERR, airspyhf_set_freq(n, 1_000_000));
            assert_eq!(ERR, airspyhf_start(n, None, std::ptr::null_mut()));
            assert_eq!(ERR, airspyhf_stop(n));
            assert_eq!(ERR, airspyhf_get_calibration(n, std::ptr::null_mut()));
        }
    }

    extern "C" fn noop_cb(_t: *mut AirspyhfTransfer) -> i32 {
        0
    }

    #[test]
    #[cfg_attr(miri, ignore)]
    fn start_stop_restart() {
        let mut handle: AirspyhfDeviceHandle = std::ptr::null_mut();
        unsafe {
            let _ = airspyhf_open(&mut handle as *mut _);
        }
        if handle.is_null() {
            return; // no hardware
        }
        unsafe {
            let res = airspyhf_start(handle, Some(noop_cb), std::ptr::null_mut());
            if res == OK {
                assert_eq!(1, airspyhf_is_streaming(handle));
                assert_eq!(0, airspyhf_stop(handle));
                assert_eq!(
                    0,
                    airspyhf_start(handle, Some(noop_cb), std::ptr::null_mut())
                );
                assert_eq!(1, airspyhf_is_streaming(handle));
                let _ = airspyhf_stop(handle);
            }
            let _ = airspyhf_close(handle);
        }
    }

    #[test]
    #[cfg_attr(miri, ignore)]
    fn close_while_streaming() {
        let mut handle: AirspyhfDeviceHandle = std::ptr::null_mut();
        unsafe {
            let _ = airspyhf_open(&mut handle as *mut _);
        }
        if handle.is_null() {
            return; // no hardware
        }
        unsafe {
            let _ = airspyhf_start(handle, Some(noop_cb), std::ptr::null_mut());
            assert_eq!(0, airspyhf_close(handle));
        }
    }
}
