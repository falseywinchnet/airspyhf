//! Rust port of the AirspyHF driver.
//!
//! This crate is currently a work in progress.  It exposes a C ABI compatible
//! interface so existing applications can link against it while the
//! implementation is gradually migrated to safe Rust.

use std::io::{self, ErrorKind};
use std::sync::Mutex;

mod iq_balancer;
use iq_balancer::IqBalancer;
const USB_VID: u16 = 0x03EB;
const USB_PID: u16 = 0x800C;
const DEFAULT_SAMPLERATE: u32 = 768_000;
const DEFAULT_ATT_STEP_COUNT: usize = 9;
const DEFAULT_ATT_STEP_INCREMENT: f32 = 6.0;

pub const AIRSPYHF_VER_MAJOR: u32 = 1;
pub const AIRSPYHF_VER_MINOR: u32 = 8;
pub const AIRSPYHF_VER_REVISION: u32 = 0;
pub const SAMPLES_TO_TRANSFER: i32 = 1024 * 4;
#[repr(C)]
pub struct AirspyhfComplexFloat {
    pub re: f32,
    pub im: f32,
}
#[repr(C)]
pub struct AirspyhfLibVersion {
    pub major_version: u32,
    pub minor_version: u32,
    pub revision: u32,
}

#[repr(C)]
pub struct AirspyhfReadPartIdSerialNo {
    pub part_id: u32,
    pub serial_no: [u32; 4],
}

use nusb::{self, Device, MaybeFuture};

/// Type signature for streaming callback.
pub type AirspyhfSampleBlockCbFn = Option<unsafe extern "C" fn(*mut std::ffi::c_void) -> i32>;

/// Opaque device handle exposed through the C API.
#[repr(C)]
pub struct AirspyHfDevice {
    /// Underlying USB handle.  Wrapped in a mutex for thread safety.
    pub(crate) handle: Mutex<Device>,
    pub(crate) streaming: bool,
    pub(crate) stop_requested: bool,
    pub(crate) is_low_if: bool,
    pub(crate) enable_dsp: u8,
    pub(crate) iq_balancer: *mut IqBalancer,
    pub(crate) freq_hz: f64,
    pub(crate) freq_khz: u32,
    pub(crate) calibration_ppb: i32,
    pub(crate) calibration_vctcxo: u16,
    pub(crate) frontend_options: u32,
    pub(crate) optimal_point: f32,
    pub(crate) current_samplerate: u32,
    pub(crate) supported_samplerates: Vec<u32>,
    pub(crate) supported_att_steps: Vec<f32>,
    pub(crate) filter_gain: f32,
    pub(crate) att_index: u8,
    pub(crate) bias_tee: i8,
    pub(crate) bias_tee_count: i32,
    pub(crate) user_output: [u8; 4],
    pub(crate) hf_agc: u8,
    pub(crate) hf_agc_threshold: u8,
    pub(crate) hf_lna: u8,
}

impl AirspyHfDevice {
    /// Attempt to open the first matching AirspyHF device.
    pub fn open_first() -> Result<Self, io::Error> {
        let di = nusb::list_devices()
            .wait()
            .map_err(io::Error::other)?
            .find(|d| d.vendor_id() == USB_VID && d.product_id() == USB_PID)
            .ok_or_else(|| io::Error::new(ErrorKind::NotFound, "AirspyHF not found"))?;
        let device = di.open().wait().map_err(io::Error::other)?;
        let bal = unsafe { iq_balancer::iq_balancer_create(0.0, 0.0) };
        Ok(AirspyHfDevice {
            handle: Mutex::new(device),
            streaming: false,
            stop_requested: false,
            is_low_if: false,
            enable_dsp: 1,
            iq_balancer: bal,
            freq_hz: 0.0,
            freq_khz: 0,
            calibration_ppb: 0,
            calibration_vctcxo: 0,
            frontend_options: 0,
            optimal_point: 0.0,
            current_samplerate: DEFAULT_SAMPLERATE,
            supported_samplerates: vec![DEFAULT_SAMPLERATE],
            supported_att_steps: (0..DEFAULT_ATT_STEP_COUNT)
                .map(|i| i as f32 * DEFAULT_ATT_STEP_INCREMENT)
                .collect(),
            filter_gain: 1.0,
            att_index: 0,
            bias_tee: 0,
            bias_tee_count: 1,
            user_output: [0; 4],
            hf_agc: 0,
            hf_agc_threshold: 0,
            hf_lna: 0,
        })
    }
}

impl AirspyHfDevice {
    pub fn open_by_serial(serial: u64) -> Result<Self, io::Error> {
        let devices = nusb::list_devices().wait().map_err(io::Error::other)?;
        for di in devices {
            if di.vendor_id() == USB_VID && di.product_id() == USB_PID {
                if let Some(sn) = di.serial_number() {
                    if let Some(hex) = sn.strip_prefix("AIRSPYHF SN:") {
                        if u64::from_str_radix(hex, 16).unwrap_or(0) == serial {
                            let dev = di.open().wait().map_err(io::Error::other)?;
                            let bal = unsafe { iq_balancer::iq_balancer_create(0.0, 0.0) };
                            return Ok(AirspyHfDevice {
                                handle: Mutex::new(dev),
                                streaming: false,
                                stop_requested: false,
                                is_low_if: false,
                                enable_dsp: 1,
                                iq_balancer: bal,
                                freq_hz: 0.0,
                                freq_khz: 0,
                                calibration_ppb: 0,
                                calibration_vctcxo: 0,
                                frontend_options: 0,
                                optimal_point: 0.0,
                                current_samplerate: DEFAULT_SAMPLERATE,
                                supported_samplerates: vec![DEFAULT_SAMPLERATE],
                                supported_att_steps: (0..DEFAULT_ATT_STEP_COUNT)
                                    .map(|i| i as f32 * DEFAULT_ATT_STEP_INCREMENT)
                                    .collect(),
                                filter_gain: 1.0,
                                att_index: 0,
                                bias_tee: 0,
                                bias_tee_count: 1,
                                user_output: [0; 4],
                                hf_agc: 0,
                                hf_agc_threshold: 0,
                                hf_lna: 0,
                            });
                        }
                    }
                }
            }
        }
        Err(io::Error::new(ErrorKind::NotFound, "AirspyHF not found"))
    }

    pub fn list_serials() -> Result<Vec<u64>, io::Error> {
        let devices = nusb::list_devices().wait().map_err(io::Error::other)?;
        let mut out = Vec::new();
        for di in devices {
            if di.vendor_id() == USB_VID && di.product_id() == USB_PID {
                if let Some(sn) = di.serial_number() {
                    if let Some(hex) = sn.strip_prefix("AIRSPYHF SN:") {
                        if let Ok(val) = u64::from_str_radix(hex, 16) {
                            out.push(val);
                        }
                    }
                }
            }
        }
        Ok(out)
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
    let d = Box::from_raw(dev);
    iq_balancer::iq_balancer_destroy(d.iq_balancer);
    // Box drops here
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

#[no_mangle]
pub unsafe extern "C" fn airspyhf_open_sn(out: *mut AirspyhfDeviceHandle, serial: u64) -> i32 {
    let dev = match AirspyHfDevice::open_by_serial(serial) {
        Ok(d) => Box::into_raw(Box::new(d)),
        Err(_) => return AirspyhfError::Error as i32,
    };
    if !out.is_null() {
        *out = dev;
    }
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_list_devices(serials: *mut u64, count: i32) -> i32 {
    match AirspyHfDevice::list_serials() {
        Ok(list) => {
            if !serials.is_null() && count > 0 {
                let len = list.len().min(count as usize);
                std::ptr::copy_nonoverlapping(list.as_ptr(), serials, len);
            }
            list.len() as i32
        }
        Err(_) => AirspyhfError::Error as i32,
    }
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_lib_version(ver: *mut AirspyhfLibVersion) {
    if ver.is_null() {
        return;
    }
    (*ver).major_version = AIRSPYHF_VER_MAJOR;
    (*ver).minor_version = AIRSPYHF_VER_MINOR;
    (*ver).revision = AIRSPYHF_VER_REVISION;
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_get_output_size(_dev: AirspyhfDeviceHandle) -> i32 {
    SAMPLES_TO_TRANSFER
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_is_low_if(dev: AirspyhfDeviceHandle) -> i32 {
    if dev.is_null() {
        return 0;
    }
    let d = &*(dev);
    if d.is_low_if {
        1
    } else {
        0
    }
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_is_streaming(dev: AirspyhfDeviceHandle) -> i32 {
    if dev.is_null() {
        return 0;
    }
    let d = &*(dev);
    if d.streaming && !d.stop_requested {
        1
    } else {
        0
    }
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_lib_dsp(dev: AirspyhfDeviceHandle, flag: u8) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let mut_ref = &mut *dev;
    mut_ref.enable_dsp = flag;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_optimal_iq_correction_point(
    dev: AirspyhfDeviceHandle,
    w: f32,
) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let mut_ref = &mut *dev;
    iq_balancer::iq_balancer_set_optimal_point(mut_ref.iq_balancer, w);
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_iq_balancer_configure(
    dev: AirspyhfDeviceHandle,
    b: i32,
    fi: i32,
    fo: i32,
    ci: i32,
) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let mut_ref = &mut *dev;
    iq_balancer::iq_balancer_configure(mut_ref.iq_balancer, b, fi, fo, ci);
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_open_fd(_out: *mut AirspyhfDeviceHandle, _fd: i32) -> i32 {
    AirspyhfError::Unsupported as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_get_samplerates(
    dev: AirspyhfDeviceHandle,
    buffer: *mut u32,
    len: u32,
) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let d = &*(dev);
    if len == 0 {
        if !buffer.is_null() {
            *buffer = d.supported_samplerates.len() as u32;
        }
    } else if (len as usize) <= d.supported_samplerates.len() {
        if !buffer.is_null() {
            std::ptr::copy_nonoverlapping(d.supported_samplerates.as_ptr(), buffer, len as usize);
        }
    } else {
        return AirspyhfError::Error as i32;
    }
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_samplerate(
    dev: AirspyhfDeviceHandle,
    samplerate: u32,
) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let d = &mut *dev;
    if d.supported_samplerates.contains(&samplerate) {
        d.current_samplerate = samplerate;
        AirspyhfError::Success as i32
    } else if (samplerate as usize) < d.supported_samplerates.len() {
        d.current_samplerate = d.supported_samplerates[samplerate as usize];
        AirspyhfError::Success as i32
    } else {
        AirspyhfError::Error as i32
    }
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_freq(dev: AirspyhfDeviceHandle, freq_hz: u32) -> i32 {
    airspyhf_set_freq_double(dev, freq_hz as f64)
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_freq_double(dev: AirspyhfDeviceHandle, freq_hz: f64) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let d = &mut *dev;
    d.freq_hz = freq_hz;
    d.freq_khz = freq_hz as u32 / 1000;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_get_frontend_options(
    dev: AirspyhfDeviceHandle,
    flags: *mut u32,
) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    if !flags.is_null() {
        *flags = (*dev).frontend_options;
        AirspyhfError::Success as i32
    } else {
        AirspyhfError::Error as i32
    }
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_frontend_options(
    dev: AirspyhfDeviceHandle,
    flags: u32,
) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    (*dev).frontend_options = flags;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_get_calibration(dev: AirspyhfDeviceHandle, ppb: *mut i32) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    if !ppb.is_null() {
        *ppb = (*dev).calibration_ppb;
        AirspyhfError::Success as i32
    } else {
        AirspyhfError::Error as i32
    }
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_calibration(dev: AirspyhfDeviceHandle, ppb: i32) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    (*dev).calibration_ppb = ppb;
    airspyhf_set_freq_double(dev, (*dev).freq_hz)
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_get_vctcxo_calibration(
    dev: AirspyhfDeviceHandle,
    vc: *mut u16,
) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    if !vc.is_null() {
        *vc = (*dev).calibration_vctcxo;
        AirspyhfError::Success as i32
    } else {
        AirspyhfError::Error as i32
    }
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_vctcxo_calibration(
    dev: AirspyhfDeviceHandle,
    vc: u16,
) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    (*dev).calibration_vctcxo = vc;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_flash_configuration(_dev: AirspyhfDeviceHandle) -> i32 {
    AirspyhfError::Unsupported as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_start(
    dev: AirspyhfDeviceHandle,
    _cb: AirspyhfSampleBlockCbFn,
    _ctx: *mut std::ffi::c_void,
) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let d = &mut *dev;
    d.streaming = true;
    d.stop_requested = false;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_stop(dev: AirspyhfDeviceHandle) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let d = &mut *dev;
    d.stop_requested = true;
    d.streaming = false;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_get_att_steps(
    dev: AirspyhfDeviceHandle,
    buffer: *mut std::ffi::c_void,
    len: u32,
) -> i32 {
    if dev.is_null() || buffer.is_null() {
        return AirspyhfError::Error as i32;
    }
    let d = &*(dev);
    if len == 0 {
        *(buffer as *mut u32) = d.supported_att_steps.len() as u32;
    } else if (len as usize) <= d.supported_att_steps.len() {
        std::ptr::copy_nonoverlapping(
            d.supported_att_steps.as_ptr(),
            buffer as *mut f32,
            len as usize,
        );
    } else {
        return AirspyhfError::Error as i32;
    }
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_att(dev: AirspyhfDeviceHandle, att: f32) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let d = &mut *dev;
    let mut idx = 0usize;
    for (i, val) in d.supported_att_steps.iter().enumerate() {
        if *val >= att {
            idx = i;
            break;
        }
    }
    d.att_index = idx as u8;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_bias_tee(dev: AirspyhfDeviceHandle, value: i8) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let d = &mut *dev;
    d.bias_tee = value;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_get_bias_tee_count(
    dev: AirspyhfDeviceHandle,
    count: *mut i32,
) -> i32 {
    if dev.is_null() || count.is_null() {
        return AirspyhfError::Error as i32;
    }
    *count = (*dev).bias_tee_count;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_get_bias_tee_name(
    _dev: AirspyhfDeviceHandle,
    _index: i32,
    name: *mut std::os::raw::c_char,
    length: u8,
) -> i32 {
    if name.is_null() || length == 0 {
        return AirspyhfError::Error as i32;
    }
    let text = b"HF Bias Tee\0";
    let copy_len = std::cmp::min((length as usize) - 1, text.len() - 1);
    std::ptr::copy_nonoverlapping(text.as_ptr() as *const i8, name, copy_len);
    *name.add(copy_len) = 0;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_board_partid_serialno_read(
    _dev: AirspyhfDeviceHandle,
    out: *mut AirspyhfReadPartIdSerialNo,
) -> i32 {
    if out.is_null() {
        return AirspyhfError::Error as i32;
    }
    (*out).part_id = 0;
    (*out).serial_no = [0; 4];
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_version_string_read(
    _dev: AirspyhfDeviceHandle,
    version: *mut std::os::raw::c_char,
    length: u8,
) -> i32 {
    if version.is_null() || length == 0 {
        return AirspyhfError::Error as i32;
    }
    let text = b"RUSTPORT 0.1\0";
    let copy_len = std::cmp::min((length as usize) - 1, text.len() - 1);
    std::ptr::copy_nonoverlapping(text.as_ptr() as *const i8, version, copy_len);
    *version.add(copy_len) = 0;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_user_output(
    dev: AirspyhfDeviceHandle,
    pin: u32,
    value: u32,
) -> i32 {
    if dev.is_null() || pin > 3 {
        return AirspyhfError::Error as i32;
    }
    let d = &mut *dev;
    d.user_output[pin as usize] = value as u8;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_hf_agc(dev: AirspyhfDeviceHandle, flag: u8) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    (*dev).hf_agc = flag;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_hf_agc_threshold(dev: AirspyhfDeviceHandle, flag: u8) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    (*dev).hf_agc_threshold = flag;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_hf_att(dev: AirspyhfDeviceHandle, index: u8) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let d = &mut *dev;
    if (index as usize) < d.supported_att_steps.len() {
        d.att_index = index;
        AirspyhfError::Success as i32
    } else {
        AirspyhfError::Error as i32
    }
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_hf_lna(dev: AirspyhfDeviceHandle, flag: u8) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    (*dev).hf_lna = flag;
    AirspyhfError::Success as i32
}
