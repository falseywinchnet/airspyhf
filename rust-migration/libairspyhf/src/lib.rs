//! Rust port of the AirspyHF driver.
//!
//! This crate is currently a work in progress.  It exposes a C ABI compatible
//! interface so existing applications can link against it while the
//! implementation is gradually migrated to safe Rust.

use std::ffi::c_void;
use std::io::{self, ErrorKind};
use std::sync::Mutex;

mod iq_balancer;
use iq_balancer::IqBalancer;
const USB_VID: u16 = 0x03EB;
const USB_PID: u16 = 0x800C;
const DEFAULT_SAMPLERATE: u32 = 768_000;
const DEFAULT_ATT_STEP_COUNT: usize = 9;
const DEFAULT_ATT_STEP_INCREMENT: f32 = 6.0;
const RAW_BUFFER_COUNT: usize = 8;

pub const AIRSPYHF_VER_MAJOR: u32 = 1;
pub const AIRSPYHF_VER_MINOR: u32 = 8;
pub const AIRSPYHF_VER_REVISION: u32 = 0;
pub const SAMPLES_TO_TRANSFER: i32 = 1024 * 4;
#[repr(C)]
#[derive(Clone, Copy)]
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

use nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};
use nusb::{self, Device, MaybeFuture};
use std::time::Duration;

const CTRL_TIMEOUT_MS: u64 = 500;

const AIRSPYHF_RECEIVER_MODE: u8 = 1;
const AIRSPYHF_SET_FREQ: u8 = 2;
const AIRSPYHF_GET_SAMPLERATES: u8 = 3;
const AIRSPYHF_SET_SAMPLERATE: u8 = 4;
const AIRSPYHF_GET_SERIALNO_BOARDID: u8 = 7;
const AIRSPYHF_SET_USER_OUTPUT: u8 = 8;
const AIRSPYHF_GET_VERSION_STRING: u8 = 9;
const AIRSPYHF_SET_AGC: u8 = 10;
const AIRSPYHF_SET_AGC_THRESHOLD: u8 = 11;
const AIRSPYHF_SET_ATT: u8 = 12;
const AIRSPYHF_SET_LNA: u8 = 13;
const AIRSPYHF_SET_VCTCXO_CALIBRATION: u8 = 17;
const AIRSPYHF_SET_FRONTEND_OPTIONS: u8 = 18;
const AIRSPYHF_GET_ATT_STEPS: u8 = 19;
const AIRSPYHF_GET_BIAS_TEE_COUNT: u8 = 20;
const AIRSPYHF_GET_BIAS_TEE_NAME: u8 = 21;
const AIRSPYHF_SET_BIAS_TEE: u8 = 22;
const MAX_VERSION_STRING_SIZE: usize = 255;

/// Type signature for streaming callback.
#[repr(C)]
pub struct AirspyhfTransfer {
    pub device: AirspyhfDeviceHandle,
    pub ctx: *mut c_void,
    pub samples: *mut AirspyhfComplexFloat,
    pub sample_count: i32,
    pub dropped_samples: u64,
}

/// Type signature for streaming callback.
pub type AirspyhfSampleBlockCbFn = Option<unsafe extern "C" fn(*mut AirspyhfTransfer) -> i32>;

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
    pub(crate) freq_delta_hz: f64,
    pub(crate) freq_shift: f64,
    pub(crate) vec: AirspyhfComplexFloat,
    pub(crate) callback: AirspyhfSampleBlockCbFn,
    pub(crate) ctx: *mut c_void,
    pub(crate) thread: Option<std::thread::JoinHandle<()>>,
    pub(crate) output_buffer: Vec<AirspyhfComplexFloat>,
    pub(crate) dropped_buffers: u64,
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
            freq_delta_hz: 0.0,
            freq_shift: 0.0,
            vec: AirspyhfComplexFloat { re: 1.0, im: 0.0 },
            callback: None,
            ctx: std::ptr::null_mut(),
            thread: None,
            output_buffer: Vec::new(),
            dropped_buffers: 0,
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
                                freq_delta_hz: 0.0,
                                freq_shift: 0.0,
                                vec: AirspyhfComplexFloat { re: 1.0, im: 0.0 },
                                callback: None,
                                ctx: std::ptr::null_mut(),
                                thread: None,
                                output_buffer: Vec::new(),
                                dropped_buffers: 0,
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

    fn vendor_out(&self, request: u8, value: u16, index: u16, data: &[u8]) -> io::Result<()> {
        let handle = self.handle.lock().unwrap().clone();
        handle
            .control_out(
                ControlOut {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index,
                    data,
                },
                Duration::from_millis(CTRL_TIMEOUT_MS),
            )
            .wait()
            .map_err(io::Error::other)
    }

    fn vendor_in(&self, request: u8, value: u16, index: u16, buf: &mut [u8]) -> io::Result<usize> {
        let handle = self.handle.lock().unwrap().clone();
        let data = handle
            .control_in(
                ControlIn {
                    control_type: ControlType::Vendor,
                    recipient: Recipient::Device,
                    request,
                    value,
                    index,
                    length: buf.len() as u16,
                },
                Duration::from_millis(CTRL_TIMEOUT_MS),
            )
            .wait()
            .map_err(io::Error::other)?;
        let len = std::cmp::min(data.len(), buf.len());
        buf[..len].copy_from_slice(&data[..len]);
        Ok(len)
    }

    fn multiply_complex_complex(a: &mut AirspyhfComplexFloat, b: &AirspyhfComplexFloat) {
        let re = a.re * b.re - a.im * b.im;
        a.im = a.im * b.re + a.re * b.im;
        a.re = re;
    }

    fn multiply_complex_real(a: &mut AirspyhfComplexFloat, b: f32) {
        a.re *= b;
        a.im *= b;
    }

    fn rotate_complex(vec: &mut AirspyhfComplexFloat, rot: &AirspyhfComplexFloat) {
        Self::multiply_complex_complex(vec, rot);
        let norm = 1.99 - (vec.re * vec.re + vec.im * vec.im);
        Self::multiply_complex_real(vec, norm as f32);
    }

    fn convert_samples(&mut self, src: &[i16]) -> usize {
        let count = src.len() / 2;
        if self.output_buffer.len() < count {
            self.output_buffer
                .resize(count, AirspyhfComplexFloat { re: 0.0, im: 0.0 });
        }
        let conv_gain = (1.0f32 / 32768.0) * self.filter_gain;
        for i in 0..count {
            let im = src[2 * i] as f32;
            let re = src[2 * i + 1] as f32;
            self.output_buffer[i].re = re * conv_gain;
            self.output_buffer[i].im = im * conv_gain;
        }
        if self.enable_dsp != 0 {
            if !self.is_low_if {
                unsafe {
                    iq_balancer::iq_balancer_process(
                        self.iq_balancer,
                        self.output_buffer.as_mut_ptr(),
                        count as i32,
                    );
                }
            }
            if self.freq_shift != 0.0 {
                let angle =
                    2.0 * std::f64::consts::PI * self.freq_shift / self.current_samplerate as f64;
                let rot = AirspyhfComplexFloat {
                    re: angle.cos() as f32,
                    im: -(angle.sin() as f32),
                };
                let mut vec = self.vec;
                for s in &mut self.output_buffer[..count] {
                    Self::rotate_complex(&mut vec, &rot);
                    Self::multiply_complex_complex(s, &vec);
                }
                self.vec = vec;
            }
        }
        count
    }

    fn start_streaming(&mut self) -> io::Result<()> {
        if self.streaming {
            return Err(io::Error::new(ErrorKind::Other, "already streaming"));
        }
        self.vendor_out(AIRSPYHF_RECEIVER_MODE, 0, 0, &[])?;
        let handle = self.handle.lock().unwrap().clone();
        let dev_ptr = self as *mut AirspyHfDevice as usize;
        let thread = std::thread::spawn(move || {
            let dev_ptr = dev_ptr as *mut AirspyHfDevice;
            let mut iface = match handle.claim_interface(0).wait() {
                Ok(i) => i,
                Err(_) => return,
            };
            let mut ep = match iface.endpoint::<nusb::transfer::Bulk, nusb::transfer::In>(0x81) {
                Ok(e) => e,
                Err(_) => return,
            };
            let buf_len = (SAMPLES_TO_TRANSFER as usize) * 4;
            for _ in 0..RAW_BUFFER_COUNT {
                let mut b = ep.allocate(buf_len);
                b.set_requested_len(buf_len);
                ep.submit(b);
            }
            loop {
                if let Some(mut c) = ep.wait_next_complete(Duration::from_millis(1000)) {
                    let d = unsafe { &mut *dev_ptr };
                    if d.stop_requested {
                        break;
                    }
                    if c.status.is_err() {
                        break;
                    }
                    let slice = unsafe {
                        std::slice::from_raw_parts(
                            c.buffer.as_ptr() as *const i16,
                            c.buffer.len() / 2,
                        )
                    };
                    let count = d.convert_samples(slice);
                    if let Some(cb) = d.callback {
                        let mut transfer = AirspyhfTransfer {
                            device: dev_ptr,
                            ctx: d.ctx,
                            samples: d.output_buffer.as_mut_ptr(),
                            sample_count: count as i32,
                            dropped_samples: d.dropped_buffers,
                        };
                        if unsafe { cb(&mut transfer as *mut _) } != 0 {
                            d.stop_requested = true;
                            break;
                        }
                    }
                    ep.submit(c.buffer);
                } else {
                    break;
                }
            }
        });
        self.thread = Some(thread);
        self.streaming = true;
        Ok(())
    }

    fn stop_streaming(&mut self) {
        self.stop_requested = true;
        if let Some(t) = self.thread.take() {
            let _ = t.join();
        }
        let _ = self.vendor_out(AIRSPYHF_RECEIVER_MODE, 0, 0, &[]);
        self.streaming = false;
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
    let target = if d.supported_samplerates.contains(&samplerate) {
        samplerate
    } else if (samplerate as usize) < d.supported_samplerates.len() {
        d.supported_samplerates[samplerate as usize]
    } else {
        return AirspyhfError::Error as i32;
    };
    if d.vendor_out(AIRSPYHF_SET_SAMPLERATE, 0, target as u16, &[])
        .is_err()
    {
        return AirspyhfError::Error as i32;
    }
    d.current_samplerate = target;
    AirspyhfError::Success as i32
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
    let freq_khz = (freq_hz as u32) / 1000;
    let buf = freq_khz.to_be_bytes();
    if d.vendor_out(AIRSPYHF_SET_FREQ, 0, 0, &buf).is_err() {
        return AirspyhfError::Error as i32;
    }
    d.freq_hz = freq_hz;
    d.freq_khz = freq_khz;
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
    let d = &mut *dev;
    if d.vendor_out(
        AIRSPYHF_SET_FRONTEND_OPTIONS,
        (flags & 0xffff) as u16,
        (flags >> 16) as u16,
        &[],
    )
    .is_err()
    {
        return AirspyhfError::Error as i32;
    }
    d.frontend_options = flags;
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
    let d = &mut *dev;
    if d.vendor_out(AIRSPYHF_SET_VCTCXO_CALIBRATION, vc, 0, &[])
        .is_err()
    {
        return AirspyhfError::Error as i32;
    }
    d.calibration_vctcxo = vc;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_flash_configuration(_dev: AirspyhfDeviceHandle) -> i32 {
    AirspyhfError::Unsupported as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_start(
    dev: AirspyhfDeviceHandle,
    cb: AirspyhfSampleBlockCbFn,
    ctx: *mut c_void,
) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let d = &mut *dev;
    d.callback = cb;
    d.ctx = ctx;
    match d.start_streaming() {
        Ok(()) => AirspyhfError::Success as i32,
        Err(_) => AirspyhfError::Error as i32,
    }
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_stop(dev: AirspyhfDeviceHandle) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let d = &mut *dev;
    d.stop_streaming();
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
    if d.vendor_out(AIRSPYHF_SET_ATT, idx as u16, 0, &[]).is_err() {
        return AirspyhfError::Error as i32;
    }
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_bias_tee(dev: AirspyhfDeviceHandle, value: i8) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let d = &mut *dev;
    if d.vendor_out(AIRSPYHF_SET_BIAS_TEE, value as u16, 0, &[])
        .is_err()
    {
        return AirspyhfError::Error as i32;
    }
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
    let d = &mut *dev;
    let mut buf = [0u8; 4];
    if d.vendor_in(AIRSPYHF_GET_BIAS_TEE_COUNT, 0, 0, &mut buf)
        .is_err()
    {
        return AirspyhfError::Error as i32;
    }
    *count = i32::from_le_bytes(buf);
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
    let mut buf = [0u8; MAX_VERSION_STRING_SIZE];
    let d = &*(_dev);
    if d.vendor_in(AIRSPYHF_GET_BIAS_TEE_NAME, 0, _index as u16, &mut buf)
        .is_err()
    {
        return AirspyhfError::Error as i32;
    }
    let copy_len = std::cmp::min((length as usize) - 1, buf.len());
    std::ptr::copy_nonoverlapping(buf.as_ptr() as *const i8, name, copy_len);
    *name.add(copy_len) = 0;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_board_partid_serialno_read(
    dev: AirspyhfDeviceHandle,
    out: *mut AirspyhfReadPartIdSerialNo,
) -> i32 {
    if out.is_null() {
        return AirspyhfError::Error as i32;
    }
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let d = &*dev;
    let mut buf = [0u8; std::mem::size_of::<AirspyhfReadPartIdSerialNo>()];
    if d.vendor_in(AIRSPYHF_GET_SERIALNO_BOARDID, 0, 0, &mut buf)
        .is_err()
    {
        return AirspyhfError::Error as i32;
    }
    std::ptr::copy_nonoverlapping(
        buf.as_ptr(),
        out as *mut u8,
        std::mem::size_of::<AirspyhfReadPartIdSerialNo>(),
    );
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_version_string_read(
    dev: AirspyhfDeviceHandle,
    version: *mut std::os::raw::c_char,
    length: u8,
) -> i32 {
    if version.is_null() || length == 0 {
        return AirspyhfError::Error as i32;
    }
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let d = &*dev;
    let mut buf = [0u8; MAX_VERSION_STRING_SIZE];
    if d.vendor_in(AIRSPYHF_GET_VERSION_STRING, 0, 0, &mut buf)
        .is_err()
    {
        return AirspyhfError::Error as i32;
    }
    let copy_len = std::cmp::min((length as usize) - 1, buf.len());
    std::ptr::copy_nonoverlapping(buf.as_ptr() as *const i8, version, copy_len);
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
    if d.vendor_out(AIRSPYHF_SET_USER_OUTPUT, pin as u16, value as u16, &[])
        .is_err()
    {
        return AirspyhfError::Error as i32;
    }
    d.user_output[pin as usize] = value as u8;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_hf_agc(dev: AirspyhfDeviceHandle, flag: u8) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let d = &mut *dev;
    if d.vendor_out(AIRSPYHF_SET_AGC, flag as u16, 0, &[]).is_err() {
        return AirspyhfError::Error as i32;
    }
    d.hf_agc = flag;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_hf_agc_threshold(dev: AirspyhfDeviceHandle, flag: u8) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let d = &mut *dev;
    if d.vendor_out(AIRSPYHF_SET_AGC_THRESHOLD, flag as u16, 0, &[])
        .is_err()
    {
        return AirspyhfError::Error as i32;
    }
    d.hf_agc_threshold = flag;
    AirspyhfError::Success as i32
}

#[no_mangle]
pub unsafe extern "C" fn airspyhf_set_hf_att(dev: AirspyhfDeviceHandle, index: u8) -> i32 {
    if dev.is_null() {
        return AirspyhfError::Error as i32;
    }
    let d = &mut *dev;
    if (index as usize) < d.supported_att_steps.len() {
        if d.vendor_out(AIRSPYHF_SET_ATT, index as u16, 0, &[])
            .is_err()
        {
            return AirspyhfError::Error as i32;
        }
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
    let d = &mut *dev;
    if d.vendor_out(AIRSPYHF_SET_LNA, flag as u16, 0, &[]).is_err() {
        return AirspyhfError::Error as i32;
    }
    d.hf_lna = flag;
    AirspyhfError::Success as i32
}
