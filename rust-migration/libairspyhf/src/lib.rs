//! Rust port of the AirspyHF driver.
//!
//! This crate is currently a work in progress.  It exposes a C ABI compatible
//! interface so existing applications can link against it while the
//! implementation is gradually migrated to safe Rust.

use std::io::{self, ErrorKind};
use std::sync::Mutex;

use nusb::{self, Device, MaybeFuture};

/// Opaque device handle exposed through the C API.
#[repr(C)]
pub struct AirspyHfDevice {
    /// Underlying USB handle.  Wrapped in a mutex for thread safety.
    pub(crate) handle: Mutex<Device>,
    /// Placeholder for additional state that will be ported from the
    /// original C driver.
    _private: (),
}

impl AirspyHfDevice {
    /// Attempt to open the first matching AirspyHF device.
    pub fn open_first() -> Result<Self, io::Error> {
        let di = nusb::list_devices()
            .wait()
            .map_err(io::Error::other)?
            .find(|d| d.vendor_id() == 0x03EB && d.product_id() == 0x800C)
            .ok_or_else(|| io::Error::new(ErrorKind::NotFound, "AirspyHF not found"))?;
        let device = di.open().wait().map_err(io::Error::other)?;
        Ok(AirspyHfDevice {
            handle: Mutex::new(device),
            _private: (),
        })
    }
}

/// FFI-safe pointer type used by the C interface.
pub type AirspyhfDeviceHandle = *mut AirspyHfDevice;

/// Error codes mirroring the original C API.
#[repr(C)]
pub enum AirspyhfError {
    Success = 0,
    Error = -1,
    Unsupported = -2,
}

/// Open the first available AirspyHF device.
///
/// # Safety
/// The caller must ensure that `out` is a valid pointer. The function will
/// write a newly allocated device handle to this location on success.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_open(out: *mut AirspyhfDeviceHandle) -> i32 {
    let device = match AirspyHfDevice::open_first() {
        Ok(d) => Box::into_raw(Box::new(d)),
        Err(_) => return AirspyhfError::Error as i32,
    };
    unsafe {
        if !out.is_null() {
            *out = device;
        }
    }
    AirspyhfError::Success as i32
}

/// Close a previously opened device.
///
/// # Safety
/// The caller must pass a handle obtained from `airspyhf_open`. The handle is
/// freed and must not be used after this call.
#[no_mangle]
pub unsafe extern "C" fn airspyhf_close(dev: AirspyhfDeviceHandle) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    unsafe {
        drop(Box::from_raw(dev));
    }
    AirspyhfError::Success as i32
}

// Placeholder for additional FFI functions.  The DSP and configuration
// routines from the original driver will be ported here.

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
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
}
