/*
Copyright (c) 2016-2023, Youssef Touil <youssef@airspy.com>
Copyright (c) 2018, Leif Asbrink <leif@sm5bsz.com>
Copyright (C) 2024, Joshuah Rainstar <joshuah.rainstar@gmail.com>
Contributions to this work were provided by OpenAI Codex, an artificial general intelligence.

All rights reserved. See lib.rs for the full BSD license text.
*/

//! Device control and streaming (mirrors the internals of `airspyhf.cpp`).
//!
//! # Threading & soundness
//!
//! The C API contract (inherited from `libairspyhf`) is that a single device
//! handle is driven from one thread at a time. The two streaming worker threads
//! therefore **never** touch [`AirspyHfDevice`]; they share only an
//! [`Arc<StreamShared>`] (atomics + a `Mutex<IqBalancer>`) and values moved into
//! them at start. This makes concurrent `&mut` aliasing impossible by
//! construction — there is exactly one owner of the device, and the workers own
//! their own buffers. The callback context is carried as plain integers, so no
//! `unsafe impl Send` is needed anywhere.
//!
//! Streaming uses the same producer/consumer split as the C++ driver: a USB
//! worker (producer) pulls bulk-IN transfers and hands raw blocks to a bounded
//! queue; a consumer converts them and fires the user callback. When the queue
//! is full the producer counts a dropped buffer instead of blocking, so a slow
//! callback never stalls USB polling and `dropped_samples` is reported faithfully.

use std::io::{self, ErrorKind};
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicU8, Ordering};
use std::sync::mpsc::{sync_channel, Receiver, RecvTimeoutError, SyncSender, TrySendError};
use std::sync::{Arc, Mutex};
use std::thread::{JoinHandle, ThreadId};
use std::time::Duration;

use nusb::transfer::{ControlIn, ControlOut, ControlType, Recipient};
use nusb::{Device, Interface, MaybeFuture};

use crate::commands::*;
use crate::iqbalancer::IqBalancer;
use crate::{AirspyhfComplexFloat, AirspyhfTransfer, SAMPLES_TO_TRANSFER};

pub(crate) const USB_VID: u16 = 0x03EB;
pub(crate) const USB_PID: u16 = 0x800C;
const DEFAULT_SAMPLERATE: u32 = 768_000;
const MAX_SAMPLERATE_INDEX: u32 = 100;
const DEFAULT_ATT_STEP_COUNT: usize = 9;
const DEFAULT_ATT_STEP_INCREMENT: f32 = 6.0;
const RAW_BUFFER_COUNT: usize = 8;
const PRODUCER_WAIT_MS: u64 = 10;
const PRODUCER_CANCEL_DRAIN_MS: u64 = 50;
const MAX_FIRMWARE_SAMPLERATES: usize = 64;
const MAX_FIRMWARE_ATT_STEPS: usize = 64;
const DEFAULT_IF_SHIFT: u32 = 5000;
const MIN_ZERO_IF_LO: u32 = 180;
const MIN_LOW_IF_LO: u32 = 84;
const INITIAL_PHASE: f32 = 0.00006;
const INITIAL_AMPLITUDE: f32 = -0.0045;
const AIRSPYHF_ENDPOINT_IN: u8 = 1;
const CTRL_TIMEOUT_MS: u64 = 500;

const SERIAL_PREFIX: &str = "AIRSPYHF SN:";

// ---------------------------------------------------------------------------
// Shared streaming state
// ---------------------------------------------------------------------------

/// State shared between the app thread and the two streaming workers.
///
/// Everything here is `Send + Sync` by construction (atomics + a `Mutex`), so no
/// `unsafe` is required to move an `Arc<StreamShared>` onto the worker threads.
/// The scalar DSP parameters are stored as bit patterns in atomics so the
/// consumer can read the latest value each block without a lock, while the app
/// thread publishes updates from the FFI setters.
pub(crate) struct StreamShared {
    pub(crate) streaming: AtomicBool,
    pub(crate) stop_requested: AtomicBool,
    /// Bumped on every stop so stale workers retire themselves.
    pub(crate) generation: AtomicU32,

    enable_dsp: AtomicU8,
    is_low_if: AtomicBool,
    current_samplerate: AtomicU32,
    filter_gain_bits: AtomicU32,
    freq_shift_bits: AtomicU64,

    pub(crate) balancer: Mutex<IqBalancer>,
}

impl StreamShared {
    fn new() -> Self {
        StreamShared {
            streaming: AtomicBool::new(false),
            stop_requested: AtomicBool::new(false),
            generation: AtomicU32::new(0),
            enable_dsp: AtomicU8::new(1),
            is_low_if: AtomicBool::new(false),
            current_samplerate: AtomicU32::new(DEFAULT_SAMPLERATE),
            filter_gain_bits: AtomicU32::new(1.0f32.to_bits()),
            freq_shift_bits: AtomicU64::new(0),
            balancer: Mutex::new(IqBalancer::new(INITIAL_PHASE, INITIAL_AMPLITUDE)),
        }
    }

    pub(crate) fn enable_dsp(&self) -> u8 {
        self.enable_dsp.load(Ordering::Relaxed)
    }
    fn set_enable_dsp(&self, v: u8) {
        self.enable_dsp.store(v, Ordering::Relaxed);
    }
    pub(crate) fn is_low_if(&self) -> bool {
        self.is_low_if.load(Ordering::Relaxed)
    }
    fn set_is_low_if(&self, v: bool) {
        self.is_low_if.store(v, Ordering::Relaxed);
    }
    fn current_samplerate(&self) -> u32 {
        self.current_samplerate.load(Ordering::Relaxed)
    }
    fn set_current_samplerate(&self, v: u32) {
        self.current_samplerate.store(v, Ordering::Relaxed);
    }
    fn filter_gain(&self) -> f32 {
        f32::from_bits(self.filter_gain_bits.load(Ordering::Relaxed))
    }
    fn set_filter_gain(&self, v: f32) {
        self.filter_gain_bits.store(v.to_bits(), Ordering::Relaxed);
    }
    fn freq_shift(&self) -> f64 {
        f64::from_bits(self.freq_shift_bits.load(Ordering::Relaxed))
    }
    fn set_freq_shift(&self, v: f64) {
        self.freq_shift_bits.store(v.to_bits(), Ordering::Relaxed);
    }
    pub(crate) fn is_streaming(&self) -> bool {
        self.streaming.load(Ordering::SeqCst) && !self.stop_requested.load(Ordering::SeqCst)
    }
}

/// Callback + opaque pointers carried onto the consumer thread.
///
/// Only plain integers and a function pointer — trivially `Send`, and the raw
/// values are never dereferenced in Rust (they are handed straight back to the C
/// callback), so there is no aliasing and no `unsafe impl Send`.
#[derive(Clone, Copy)]
struct CallbackCtx {
    cb: crate::AirspyhfSampleBlockCbFn,
    ctx: usize,
    dev_ptr: usize,
}

/// One completed raw buffer plus the number of buffers dropped before it.
struct RawBlock {
    data: Vec<i16>,
    dropped: u32,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct FlashConfig {
    magic_number: u32,
    calibration_ppb: i32,
    calibration_vctcxo: i32,
    frontend_options: u32,
}

/// Internal representation of an Airspy HF+ receiver, exposed to C as an opaque
/// pointer. Owned by exactly one thread at a time (the C thread-safety contract);
/// derives `Send` automatically — no `unsafe impl` needed.
pub struct AirspyHfDevice {
    handle: Device,
    /// Claimed interface 0, lazily initialised. `Mutex` provides interior
    /// mutability for the `&self` control paths (single-threaded in practice).
    interface: Mutex<Option<Interface>>,

    pub(crate) shared: Arc<StreamShared>,

    freq_hz: f64,
    freq_khz: u32,
    calibration_ppb: i32,
    calibration_vctcxo: u16,
    frontend_options: u32,
    optimal_point: f32,
    supported_samplerates: Vec<u32>,
    samplerate_architectures: Vec<u8>,
    supported_att_steps: Vec<f32>,
    att_index: u8,
    bias_tee: i8,
    user_output: [u8; 4],
    hf_agc: u8,
    hf_agc_threshold: u8,
    hf_lna: u8,
    freq_delta_hz: f64,

    producer: Option<JoinHandle<()>>,
    consumer: Option<JoinHandle<()>>,
    /// Id of the consumer thread (the one that runs the callback), so a stop
    /// requested from inside the callback does not try to join itself.
    consumer_thread_id: Option<ThreadId>,
    /// Callback + ctx last passed to `start`, so a live samplerate change can
    /// restart streaming transparently (matches the C++ behaviour). `ctx` is
    /// stored as an integer and only handed back to C, never dereferenced here.
    saved_cb: crate::AirspyhfSampleBlockCbFn,
    saved_ctx: usize,
}

impl AirspyHfDevice {
    /// Wrap an opened `nusb::Device` and prime cached state from the firmware,
    /// mirroring `airspyhf_open_init`.
    fn from_device(device: Device) -> Self {
        let mut dev = AirspyHfDevice {
            handle: device,
            interface: Mutex::new(None),
            shared: Arc::new(StreamShared::new()),
            freq_hz: 0.0,
            freq_khz: 0,
            calibration_ppb: 0,
            calibration_vctcxo: 0,
            frontend_options: 0,
            optimal_point: 0.0,
            supported_samplerates: vec![DEFAULT_SAMPLERATE],
            samplerate_architectures: vec![0],
            supported_att_steps: (0..DEFAULT_ATT_STEP_COUNT)
                .map(|i| i as f32 * DEFAULT_ATT_STEP_INCREMENT)
                .collect(),
            att_index: 0,
            bias_tee: 0,
            user_output: [0; 4],
            hf_agc: 0,
            hf_agc_threshold: 0,
            hf_lna: 0,
            freq_delta_hz: 0.0,
            producer: None,
            consumer: None,
            consumer_thread_id: None,
            saved_cb: None,
            saved_ctx: 0,
        };
        dev.prime();
        dev
    }

    /// Read flash calibration then query the firmware for sample rates,
    /// attenuator steps and filter gain, applying stored calibration to the
    /// hardware exactly like the C++ open path.
    fn prime(&mut self) {
        if self.read_flash_config().is_ok() {
            let vc = self.calibration_vctcxo;
            let fo = self.frontend_options;
            let _ = self.vendor_out(AIRSPYHF_SET_VCTCXO_CALIBRATION, vc, 0, &[]);
            let _ = self.vendor_out(
                AIRSPYHF_SET_FRONTEND_OPTIONS,
                (fo & 0xffff) as u16,
                (fo >> 16) as u16,
                &[],
            );
        }
        let _ = self.fetch_samplerates();
        let _ = self.fetch_att_steps();
        let _ = self.fetch_filter_gain();
    }

    pub fn open_first() -> Result<Self, io::Error> {
        let di = nusb::list_devices()
            .wait()
            .map_err(io::Error::other)?
            .find(|d| d.vendor_id() == USB_VID && d.product_id() == USB_PID)
            .ok_or_else(|| io::Error::new(ErrorKind::NotFound, "AirspyHF not found"))?;
        let device = di.open().wait().map_err(io::Error::other)?;
        Ok(Self::from_device(device))
    }

    pub fn open_by_serial(serial: u64) -> Result<Self, io::Error> {
        let devices = nusb::list_devices().wait().map_err(io::Error::other)?;
        for di in devices {
            if di.vendor_id() == USB_VID && di.product_id() == USB_PID {
                if let Some(val) = di.serial_number().and_then(parse_serial) {
                    if val == serial {
                        let device = di.open().wait().map_err(io::Error::other)?;
                        return Ok(Self::from_device(device));
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
                if let Some(val) = di.serial_number().and_then(parse_serial) {
                    out.push(val);
                }
            }
        }
        Ok(out)
    }

    #[cfg(any(target_os = "linux", target_os = "android"))]
    pub fn open_from_fd(fd: std::os::fd::OwnedFd) -> Result<Self, io::Error> {
        let device = Device::from_fd(fd).wait().map_err(io::Error::other)?;
        Ok(Self::from_device(device))
    }

    // ---- USB control transfers -------------------------------------------

    pub(crate) fn vendor_out(
        &self,
        request: u8,
        value: u16,
        index: u16,
        data: &[u8],
    ) -> io::Result<()> {
        let ctrl = ControlOut {
            control_type: ControlType::Vendor,
            recipient: Recipient::Device,
            request,
            value,
            index,
            data,
        };
        let timeout = Duration::from_millis(CTRL_TIMEOUT_MS);
        #[cfg(target_os = "windows")]
        {
            let iface = self.claim_interface_zero()?;
            iface
                .control_out(ctrl, timeout)
                .wait()
                .map_err(io::Error::other)
        }
        #[cfg(not(target_os = "windows"))]
        {
            self.handle
                .control_out(ctrl, timeout)
                .wait()
                .map_err(io::Error::other)
        }
    }

    pub(crate) fn vendor_in(
        &self,
        request: u8,
        value: u16,
        index: u16,
        buf: &mut [u8],
    ) -> io::Result<usize> {
        let ctrl = ControlIn {
            control_type: ControlType::Vendor,
            recipient: Recipient::Device,
            request,
            value,
            index,
            length: buf.len() as u16,
        };
        let timeout = Duration::from_millis(CTRL_TIMEOUT_MS);
        let data = {
            #[cfg(target_os = "windows")]
            {
                let iface = self.claim_interface_zero()?;
                iface
                    .control_in(ctrl, timeout)
                    .wait()
                    .map_err(io::Error::other)?
            }
            #[cfg(not(target_os = "windows"))]
            {
                self.handle
                    .control_in(ctrl, timeout)
                    .wait()
                    .map_err(io::Error::other)?
            }
        };
        let len = data.len().min(buf.len());
        buf[..len].copy_from_slice(&data[..len]);
        Ok(len)
    }

    fn claim_interface_zero(&self) -> io::Result<Interface> {
        let mut guard = self.interface.lock().unwrap();
        if guard.is_none() {
            let iface = self
                .handle
                .claim_interface(0)
                .wait()
                .map_err(io::Error::other)?;
            *guard = Some(iface);
        }
        Ok(guard.as_ref().unwrap().clone())
    }

    fn clear_halt_ep(&self, ep: u8) {
        let ctrl = ControlOut {
            control_type: ControlType::Standard,
            recipient: Recipient::Endpoint,
            request: 1, // CLEAR_FEATURE
            value: 0,   // ENDPOINT_HALT
            index: ep as u16,
            data: &[],
        };
        let timeout = Duration::from_millis(CTRL_TIMEOUT_MS);
        #[cfg(target_os = "windows")]
        {
            if let Ok(guard) = self.interface.lock() {
                if let Some(iface) = guard.as_ref() {
                    let _ = iface.control_out(ctrl, timeout).wait();
                }
            }
        }
        #[cfg(not(target_os = "windows"))]
        {
            let _ = self.handle.control_out(ctrl, timeout).wait();
        }
    }

    // ---- Firmware queries -------------------------------------------------

    fn read_flash_config(&mut self) -> io::Result<()> {
        let mut buf = [0u8; 256];
        self.vendor_in(AIRSPYHF_CONFIG_READ, 0, 0, &mut buf)?;
        let cfg = FlashConfig {
            magic_number: u32::from_le_bytes(buf[0..4].try_into().unwrap()),
            calibration_ppb: i32::from_le_bytes(buf[4..8].try_into().unwrap()),
            calibration_vctcxo: i32::from_le_bytes(buf[8..12].try_into().unwrap()),
            frontend_options: u32::from_le_bytes(buf[12..16].try_into().unwrap()),
        };
        if cfg.magic_number == CALIBRATION_MAGIC {
            self.calibration_ppb = cfg.calibration_ppb;
            self.calibration_vctcxo = cfg.calibration_vctcxo as u16;
            self.frontend_options = cfg.frontend_options;
        }
        Ok(())
    }

    fn write_flash_config(&self) -> io::Result<()> {
        let mut buf = [0u8; 256];
        buf[0..4].copy_from_slice(&CALIBRATION_MAGIC.to_le_bytes());
        buf[4..8].copy_from_slice(&self.calibration_ppb.to_le_bytes());
        buf[8..12].copy_from_slice(&(self.calibration_vctcxo as i32).to_le_bytes());
        buf[12..16].copy_from_slice(&self.frontend_options.to_le_bytes());
        self.vendor_out(AIRSPYHF_CONFIG_WRITE, 0, 0, &buf)
    }

    fn fetch_samplerates(&mut self) -> io::Result<()> {
        let mut cnt = [0u8; 4];
        self.vendor_in(AIRSPYHF_GET_SAMPLERATES, 0, 0, &mut cnt)?;
        let count = u32::from_le_bytes(cnt) as usize;
        if count == 0 {
            return Err(io::Error::other("no samplerates"));
        }
        if count > MAX_FIRMWARE_SAMPLERATES {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                "too many samplerates",
            ));
        }
        let mut buf = vec![0u8; count * 4];
        self.vendor_in(AIRSPYHF_GET_SAMPLERATES, 0, count as u16, &mut buf)?;
        self.supported_samplerates = buf
            .chunks_exact(4)
            .map(|b| u32::from_le_bytes(b.try_into().unwrap()))
            .collect();
        let mut arch = vec![0u8; count];
        if self
            .vendor_in(
                AIRSPYHF_GET_SAMPLERATE_ARCHITECTURES,
                0,
                count as u16,
                &mut arch,
            )
            .is_err()
        {
            arch.fill(0);
        }
        self.samplerate_architectures = arch;
        self.shared
            .set_current_samplerate(self.supported_samplerates[0]);
        self.shared
            .set_is_low_if(self.samplerate_architectures.first().copied().unwrap_or(0) != 0);
        Ok(())
    }

    fn fetch_att_steps(&mut self) -> io::Result<()> {
        let mut cnt = [0u8; 4];
        self.vendor_in(AIRSPYHF_GET_ATT_STEPS, 0, 0, &mut cnt)?;
        let count = u32::from_le_bytes(cnt) as usize;
        if count > MAX_FIRMWARE_ATT_STEPS {
            return Err(io::Error::new(
                ErrorKind::InvalidData,
                "too many attenuation steps",
            ));
        }
        let mut buf = vec![0u8; count * 4];
        self.vendor_in(AIRSPYHF_GET_ATT_STEPS, 0, count as u16, &mut buf)?;
        self.supported_att_steps = buf
            .chunks_exact(4)
            .map(|b| f32::from_le_bytes(b.try_into().unwrap()))
            .collect();
        if self.supported_att_steps.is_empty() {
            self.supported_att_steps = (0..DEFAULT_ATT_STEP_COUNT)
                .map(|i| i as f32 * DEFAULT_ATT_STEP_INCREMENT)
                .collect();
        }
        Ok(())
    }

    fn fetch_filter_gain(&mut self) -> io::Result<()> {
        let mut buf = [0u8; 1];
        self.vendor_in(AIRSPYHF_GET_FILTER_GAIN, 0, 0, &mut buf)?;
        self.shared
            .set_filter_gain(10f32.powf(-(buf[0] as f32) * 0.05));
        Ok(())
    }

    // ---- Streaming: producer / consumer ----------------------------------

    pub(crate) fn start_streaming(
        &mut self,
        cb: crate::AirspyhfSampleBlockCbFn,
        ctx: *mut std::ffi::c_void,
    ) -> io::Result<()> {
        self.reap_finished_workers();
        if self.shared.streaming.swap(true, Ordering::SeqCst) {
            return Err(io::Error::other("already streaming"));
        }
        self.saved_cb = cb;
        self.saved_ctx = ctx as usize;
        self.shared.stop_requested.store(false, Ordering::SeqCst);
        // Reset the balancer for a fresh capture.
        if let Ok(mut bal) = self.shared.balancer.lock() {
            bal.set_optimal_point(self.optimal_point);
        }

        // receiver OFF -> claim -> clear halt -> receiver ON (matches C++ start).
        if let Err(e) = self.vendor_out(AIRSPYHF_RECEIVER_MODE, RECEIVER_MODE_OFF, 0, &[]) {
            self.shared.streaming.store(false, Ordering::SeqCst);
            return Err(e);
        }
        let iface = match self.claim_interface_zero() {
            Ok(i) => i,
            Err(e) => {
                self.shared.streaming.store(false, Ordering::SeqCst);
                return Err(e);
            }
        };
        self.clear_halt_ep(0x80 | AIRSPYHF_ENDPOINT_IN);
        if let Err(e) = self.vendor_out(AIRSPYHF_RECEIVER_MODE, RECEIVER_MODE_ON, 0, &[]) {
            self.shared.streaming.store(false, Ordering::SeqCst);
            return Err(e);
        }

        let my_gen = self.shared.generation.load(Ordering::Acquire);
        let cbctx = CallbackCtx {
            cb,
            ctx: ctx as usize,
            dev_ptr: self as *mut AirspyHfDevice as usize,
        };

        // Queue of completed blocks + a recycling pool of buffers.
        let (full_tx, full_rx) = sync_channel::<RawBlock>(RAW_BUFFER_COUNT);
        let (empty_tx, empty_rx) = sync_channel::<Vec<i16>>(RAW_BUFFER_COUNT);
        let buf_samples = (SAMPLES_TO_TRANSFER as usize) * 2; // I + Q i16 per complex sample
        for _ in 0..RAW_BUFFER_COUNT {
            let _ = empty_tx.try_send(Vec::with_capacity(buf_samples));
        }

        let shared_c = Arc::clone(&self.shared);
        let consumer =
            std::thread::spawn(move || consumer_proc(shared_c, my_gen, cbctx, full_rx, empty_tx));
        self.consumer_thread_id = Some(consumer.thread().id());

        let shared_p = Arc::clone(&self.shared);
        let producer =
            std::thread::spawn(move || producer_proc(shared_p, my_gen, iface, full_tx, empty_rx));

        self.producer = Some(producer);
        self.consumer = Some(consumer);
        Ok(())
    }

    pub(crate) fn stop_streaming(&mut self) {
        if !self.shared.streaming.load(Ordering::SeqCst)
            && self.producer.is_none()
            && self.consumer.is_none()
        {
            return;
        }
        self.shared.stop_requested.store(true, Ordering::SeqCst);
        let _ = self.vendor_out(AIRSPYHF_RECEIVER_MODE, RECEIVER_MODE_OFF, 0, &[]);
        self.shared.generation.fetch_add(1, Ordering::AcqRel);

        let on_consumer = self.consumer_thread_id == Some(std::thread::current().id());

        // Producer is never the current thread; joining it drops the queue
        // sender and unblocks the consumer.
        if let Some(t) = self.producer.take() {
            let _ = t.join();
        }
        if on_consumer {
            // Called from within the callback: detach the current consumer
            // handle and leave the generation mismatch/channel disconnect to
            // make this thread unwind itself after the callback returns.
            let _ = self.consumer.take();
            self.shared.streaming.store(false, Ordering::SeqCst);
            self.consumer_thread_id = None;
            return;
        }
        if let Some(t) = self.consumer.take() {
            let _ = t.join();
        }
        self.shared.streaming.store(false, Ordering::SeqCst);
        self.shared.stop_requested.store(false, Ordering::SeqCst);
        self.consumer_thread_id = None;
    }

    fn reap_finished_workers(&mut self) {
        if self
            .producer
            .as_ref()
            .is_some_and(std::thread::JoinHandle::is_finished)
        {
            if let Some(t) = self.producer.take() {
                let _ = t.join();
            }
        }
        if self
            .consumer
            .as_ref()
            .is_some_and(std::thread::JoinHandle::is_finished)
        {
            if let Some(t) = self.consumer.take() {
                let _ = t.join();
            }
            self.consumer_thread_id = None;
        }
    }

    pub(crate) fn is_streaming(&self) -> bool {
        self.shared.is_streaming()
    }

    pub(crate) fn is_low_if(&self) -> bool {
        self.shared.is_low_if()
    }

    pub(crate) fn set_enable_dsp(&mut self, flag: u8) {
        self.shared.set_enable_dsp(flag);
    }

    pub(crate) fn flash_configuration(&self) -> io::Result<()> {
        if self.shared.streaming.load(Ordering::SeqCst) {
            return Err(io::Error::other("streaming"));
        }
        self.write_flash_config()
    }

    // ---- Accessors for FFI getters ---------------------------------------

    pub(crate) fn supported_samplerates(&self) -> &[u32] {
        &self.supported_samplerates
    }
    pub(crate) fn supported_att_steps(&self) -> &[f32] {
        &self.supported_att_steps
    }
    pub(crate) fn calibration_ppb(&self) -> i32 {
        self.calibration_ppb
    }
    pub(crate) fn calibration_vctcxo(&self) -> u16 {
        self.calibration_vctcxo
    }
    pub(crate) fn frontend_options(&self) -> u32 {
        self.frontend_options
    }

    // ---- Parameter setters mirroring the C API ---------------------------

    pub(crate) fn set_samplerate(&mut self, samplerate: u32) -> io::Result<()> {
        let idx_usize = resolve_samplerate_index(samplerate, &self.supported_samplerates)
            .ok_or_else(|| io::Error::new(ErrorKind::InvalidInput, "unknown samplerate"))?;
        let idx = idx_usize as u16;
        self.claim_interface_zero()?;

        let was_streaming = self.shared.streaming.load(Ordering::SeqCst);
        if was_streaming {
            self.stop_streaming();
        }
        self.shared
            .set_current_samplerate(self.supported_samplerates[idx_usize]);
        self.shared.set_is_low_if(
            self.samplerate_architectures
                .get(idx_usize)
                .copied()
                .unwrap_or(0)
                != 0,
        );
        self.clear_halt_ep(0x80 | AIRSPYHF_ENDPOINT_IN);
        self.vendor_out(AIRSPYHF_SET_SAMPLERATE, 0, idx, &[])?;
        let _ = self.fetch_filter_gain();
        let freq = self.freq_hz;
        self.set_freq(freq)?;
        if was_streaming {
            let cb = self.saved_cb;
            let ctx = self.saved_ctx as *mut std::ffi::c_void;
            self.start_streaming(cb, ctx)?;
        }
        Ok(())
    }

    pub(crate) fn set_freq(&mut self, freq_hz: f64) -> io::Result<()> {
        let if_shift = if self.shared.enable_dsp() != 0 && !self.shared.is_low_if() {
            DEFAULT_IF_SHIFT as f64
        } else {
            0.0
        };
        let adjusted = adjusted_freq_hz(freq_hz, self.calibration_ppb);
        let lo_low = if self.shared.is_low_if() {
            MIN_LOW_IF_LO
        } else {
            MIN_ZERO_IF_LO
        };
        let freq_khz = freq_khz_from(adjusted, if_shift, lo_low);

        if self.freq_khz != freq_khz {
            self.vendor_out(AIRSPYHF_SET_FREQ, 0, 0, &freq_khz.to_be_bytes())?;
            let mut buf = [0u8; 4];
            if self
                .vendor_in(AIRSPYHF_GET_FREQ_DELTA, 0, 0, &mut buf)
                .is_ok()
            {
                let val =
                    (((buf[3] as i8 as i32) << 16) | ((buf[2] as i32) << 8) | buf[1] as i32) as f64;
                self.freq_delta_hz = val * 1e3 / ((1u32 << buf[0]) as f64);
            }
            self.freq_khz = freq_khz;
            if let Ok(mut bal) = self.shared.balancer.lock() {
                bal.set_optimal_point(self.optimal_point);
            }
        }

        self.freq_hz = freq_hz;
        self.shared
            .set_freq_shift(adjusted - freq_khz as f64 * 1e3 + self.freq_delta_hz);
        Ok(())
    }

    pub(crate) fn set_calibration(&mut self, ppb: i32) -> io::Result<()> {
        self.calibration_ppb = ppb;
        let freq = self.freq_hz;
        self.set_freq(freq)
    }

    pub(crate) fn set_vctcxo_calibration(&mut self, vc: u16) -> io::Result<()> {
        self.vendor_out(AIRSPYHF_SET_VCTCXO_CALIBRATION, vc, 0, &[])?;
        self.calibration_vctcxo = vc;
        Ok(())
    }

    pub(crate) fn set_frontend_options(&mut self, flags: u32) -> io::Result<()> {
        self.vendor_out(
            AIRSPYHF_SET_FRONTEND_OPTIONS,
            (flags & 0xffff) as u16,
            (flags >> 16) as u16,
            &[],
        )?;
        self.frontend_options = flags;
        Ok(())
    }

    pub(crate) fn set_att(&mut self, att: f32) -> io::Result<()> {
        let idx = att_index_for(att, &self.supported_att_steps);
        self.att_index = idx as u8;
        self.vendor_out(AIRSPYHF_SET_ATT, idx as u16, 0, &[])
    }

    pub(crate) fn set_hf_att(&mut self, index: u8) -> io::Result<()> {
        if (index as usize) >= self.supported_att_steps.len() {
            return Err(io::Error::new(ErrorKind::InvalidInput, "att index"));
        }
        self.vendor_out(AIRSPYHF_SET_ATT, index as u16, 0, &[])?;
        self.att_index = index;
        Ok(())
    }

    pub(crate) fn set_bias_tee(&mut self, value: i8) -> io::Result<()> {
        self.vendor_out(AIRSPYHF_SET_BIAS_TEE, value as u16, 0, &[])?;
        self.bias_tee = value;
        Ok(())
    }

    pub(crate) fn set_hf_agc(&mut self, flag: u8) -> io::Result<()> {
        self.vendor_out(AIRSPYHF_SET_AGC, flag as u16, 0, &[])?;
        self.hf_agc = flag;
        Ok(())
    }

    pub(crate) fn set_hf_agc_threshold(&mut self, flag: u8) -> io::Result<()> {
        self.vendor_out(AIRSPYHF_SET_AGC_THRESHOLD, flag as u16, 0, &[])?;
        self.hf_agc_threshold = flag;
        Ok(())
    }

    pub(crate) fn set_hf_lna(&mut self, flag: u8) -> io::Result<()> {
        self.vendor_out(AIRSPYHF_SET_LNA, flag as u16, 0, &[])?;
        self.hf_lna = flag;
        Ok(())
    }

    pub(crate) fn set_user_output(&mut self, pin: u32, value: u32) -> io::Result<()> {
        if pin > 3 {
            return Err(io::Error::new(ErrorKind::InvalidInput, "pin"));
        }
        self.vendor_out(AIRSPYHF_SET_USER_OUTPUT, pin as u16, value as u16, &[])?;
        self.user_output[pin as usize] = value as u8;
        Ok(())
    }

    pub(crate) fn set_iq_correction_point(&mut self, w: f32) {
        self.optimal_point = w;
        if let Ok(mut bal) = self.shared.balancer.lock() {
            bal.set_optimal_point(w);
        }
    }

    pub(crate) fn iq_balancer_configure(&mut self, b: i32, fi: i32, fo: i32, ci: i32) {
        if let Ok(mut bal) = self.shared.balancer.lock() {
            bal.configure(b, fi, fo, ci);
        }
    }
}

impl Drop for AirspyHfDevice {
    fn drop(&mut self) {
        if self.shared.streaming.load(Ordering::SeqCst) {
            self.stop_streaming();
        }
        // IqBalancer is owned by StreamShared (RAII) — nothing manual to free.
    }
}

/// Parse an `"AIRSPYHF SN:<hex>"` serial string into a u64.
fn parse_serial(sn: &str) -> Option<u64> {
    sn.strip_prefix(SERIAL_PREFIX)
        .and_then(|hex| u64::from_str_radix(hex.trim(), 16).ok())
}

// ---------------------------------------------------------------------------
// Pure helpers (no hardware / no I/O): unit-tested, Miri-run and Kani-proven.
// ---------------------------------------------------------------------------

/// Resolve a samplerate argument to an index into `rates`. Values greater than
/// `MAX_SAMPLERATE_INDEX` are treated as a rate in Hz and looked up by value;
/// smaller values are treated as a direct index. Returns `None` if the value is
/// unknown or the index is out of range. Never panics.
pub(crate) fn resolve_samplerate_index(samplerate: u32, rates: &[u32]) -> Option<usize> {
    let idx = if samplerate > MAX_SAMPLERATE_INDEX {
        rates.iter().position(|&r| r == samplerate)?
    } else {
        samplerate as usize
    };
    (idx < rates.len()).then_some(idx)
}

/// Index of the first attenuator step `>= att`, or 0 if none qualifies (matches
/// the C++ behaviour). Never panics.
pub(crate) fn att_index_for(att: f32, steps: &[f32]) -> usize {
    steps.iter().position(|&v| v >= att).unwrap_or(0)
}

/// Frequency after applying the ppb calibration.
fn adjusted_freq_hz(freq_hz: f64, calibration_ppb: i32) -> f64 {
    freq_hz * (1.0e9 + calibration_ppb as f64) * 1.0e-9
}

/// Tuning frequency in kHz: `(adjusted + if_shift)` rounded to kHz and clamped up
/// to `lo_low`. Pathological floats are handled explicitly so the command is
/// panic-free and easy for bounded model checkers to reason about.
fn freq_khz_from(adjusted: f64, if_shift: f64, lo_low: u32) -> u32 {
    let khz = ((adjusted + if_shift) * 1e-3).round();
    if khz.is_nan() {
        return lo_low;
    }
    if !khz.is_finite() {
        return if khz.is_sign_positive() {
            u32::MAX
        } else {
            lo_low
        };
    }
    if khz <= lo_low as f64 {
        lo_low
    } else if khz >= u32::MAX as f64 {
        u32::MAX
    } else {
        khz as u32
    }
}

#[cfg(test)]
#[allow(clippy::items_after_test_module)]
mod pure_tests {
    use super::*;
    use proptest::prelude::*;

    #[test]
    fn samplerate_index_by_value() {
        let rates = [912_000u32, 768_000, 384_000];
        assert_eq!(resolve_samplerate_index(768_000, &rates), Some(1));
        assert_eq!(resolve_samplerate_index(999_000, &rates), None);
    }

    #[test]
    fn samplerate_index_by_index() {
        let rates = [912_000u32, 768_000, 384_000];
        assert_eq!(resolve_samplerate_index(0, &rates), Some(0));
        assert_eq!(resolve_samplerate_index(2, &rates), Some(2));
        assert_eq!(resolve_samplerate_index(3, &rates), None); // out of range
    }

    #[test]
    fn att_index_selection() {
        let steps = [0.0f32, 6.0, 12.0, 18.0];
        assert_eq!(att_index_for(-1.0, &steps), 0);
        assert_eq!(att_index_for(6.0, &steps), 1);
        assert_eq!(att_index_for(7.0, &steps), 2);
        assert_eq!(att_index_for(1000.0, &steps), 0); // none qualifies -> 0
        assert_eq!(att_index_for(f32::NAN, &steps), 0); // no panic
    }

    #[test]
    fn freq_khz_clamps_and_saturates() {
        assert_eq!(freq_khz_from(10_000_000.0, 5000.0, 180), 10_005);
        assert_eq!(freq_khz_from(1_000.0, 0.0, 180), 180); // clamped up
                                                           // Pathological floats must not panic and must clamp to lo_low.
        assert_eq!(freq_khz_from(f64::NAN, 0.0, 180), 180);
        assert_eq!(freq_khz_from(f64::INFINITY, 0.0, 180), u32::MAX);
        assert_eq!(freq_khz_from(-1e30, 0.0, 180), 180);
    }

    #[test]
    fn adjusted_freq_applies_ppb() {
        assert!((adjusted_freq_hz(1_000_000.0, 0) - 1_000_000.0).abs() < 1e-6);
        assert!(adjusted_freq_hz(1_000_000.0, 1_000_000).is_finite());
    }

    #[test]
    fn serial_parse() {
        assert_eq!(parse_serial("AIRSPYHF SN:DEADBEEF"), Some(0xDEAD_BEEF));
        assert_eq!(parse_serial("nonsense"), None);
        assert_eq!(parse_serial("AIRSPYHF SN:zzzz"), None); // no panic
    }

    #[test]
    fn convert_samples_decodes_wire_order_without_dsp() {
        let shared = StreamShared::new();
        shared.set_enable_dsp(0);
        shared.set_filter_gain(1.0);
        let mut output = Vec::new();
        let mut rot = AirspyhfComplexFloat { re: 1.0, im: 0.0 };
        let src = [-32768i16, 32767, 16384, -16384];

        let n = convert_samples(&shared, &mut output, &mut rot, &src);

        assert_eq!(n, 2);
        assert!((output[0].re - (32767.0 / 32768.0)).abs() < f32::EPSILON);
        assert_eq!(output[0].im, -1.0);
        assert_eq!(output[1].re, -0.5);
        assert_eq!(output[1].im, 0.5);
    }

    proptest! {
        #[test]
        #[cfg_attr(miri, ignore)]
        fn samplerate_resolution_returns_only_in_bounds(
            rates in proptest::collection::vec(any::<u32>(), 0..16),
            samplerate in any::<u32>(),
        ) {
            if let Some(idx) = resolve_samplerate_index(samplerate, &rates) {
                prop_assert!(idx < rates.len());
                if samplerate > MAX_SAMPLERATE_INDEX {
                    prop_assert_eq!(rates[idx], samplerate);
                } else {
                    prop_assert_eq!(idx, samplerate as usize);
                }
            }
        }

        #[test]
        #[cfg_attr(miri, ignore)]
        fn attenuation_index_is_in_bounds_or_zero(
            steps in proptest::collection::vec(-120.0f32..120.0, 0..16),
            att in any::<f32>(),
        ) {
            let idx = att_index_for(att, &steps);
            prop_assert!(idx == 0 || idx < steps.len());
            if idx < steps.len() && steps[idx] >= att {
                prop_assert!(steps[..idx].iter().all(|v| *v < att));
            }
        }

        #[test]
        #[cfg_attr(miri, ignore)]
        fn frequency_khz_is_clamped_for_finite_inputs(
            adjusted in -1.0e12f64..1.0e12,
            if_shift in -1.0e9f64..1.0e9,
            lo_low in 0u32..10_000,
        ) {
            let khz = freq_khz_from(adjusted, if_shift, lo_low);
            prop_assert!(khz >= lo_low);
        }
    }
}

/// Kani bounded model-checking proofs (compiled only under `cargo kani`).
#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// `resolve_samplerate_index` never panics and any returned index is in
    /// bounds, for every input value and every rates list up to 8 entries.
    #[kani::proof]
    fn resolve_index_is_in_bounds() {
        let mut rates = [0u32; 8];
        for r in &mut rates {
            *r = kani::any();
        }
        let sr: u32 = kani::any();
        if let Some(idx) = resolve_samplerate_index(sr, &rates) {
            assert!(idx < rates.len());
        }
        assert_eq!(resolve_samplerate_index(0, &[]), None);
    }

    /// `att_index_for` never panics and returns an in-bounds index for any
    /// 8-entry step list and any (incl. NaN) threshold.
    #[kani::proof]
    fn att_index_is_in_bounds() {
        let mut steps = [0f32; 8];
        for s in &mut steps {
            *s = kani::any();
        }
        let att: f32 = kani::any();
        let idx = att_index_for(att, &steps);
        assert!(idx < steps.len());
        assert_eq!(att_index_for(att, &[]), 0);
    }

    /// `freq_khz_from` never panics and always returns a value `>= lo_low` for
    /// any float inputs (including NaN / infinities). Proves the saturating cast
    /// and clamp keep the tuner command well-defined.
    #[kani::proof]
    fn freq_khz_is_clamped() {
        let adjusted: f64 = kani::any();
        let if_shift: f64 = kani::any();
        let lo_low: u32 = kani::any();
        kani::assume(adjusted.is_finite());
        kani::assume(if_shift.is_finite());
        let khz = freq_khz_from(adjusted, if_shift, lo_low);
        assert!(khz >= lo_low);
    }
}

#[cfg(all(test, loom))]
mod loom_tests {
    use loom::sync::atomic::{AtomicBool, AtomicU32, Ordering};
    use loom::sync::Arc;
    use loom::thread;

    struct ModelShared {
        streaming: AtomicBool,
        stop_requested: AtomicBool,
        generation: AtomicU32,
    }

    /// Exhaustively model the stop protocol used by the real producer/consumer:
    /// a stop request is published, the generation changes, the producer may
    /// publish at most one already-completed block, the consumer observes the
    /// stop/generation, and the app thread joins both workers before clearing
    /// `streaming`.
    #[test]
    fn stop_protocol_retires_workers() {
        loom::model(|| {
            let shared = Arc::new(ModelShared {
                streaming: AtomicBool::new(true),
                stop_requested: AtomicBool::new(false),
                generation: AtomicU32::new(0),
            });
            let my_gen = shared.generation.load(Ordering::Acquire);
            let block_available = Arc::new(AtomicBool::new(false));

            let producer_shared = Arc::clone(&shared);
            let producer_block = Arc::clone(&block_available);
            let producer = thread::spawn(move || {
                if producer_shared.generation.load(Ordering::Acquire) == my_gen
                    && !producer_shared.stop_requested.load(Ordering::Relaxed)
                {
                    producer_block.store(true, Ordering::Release);
                }
            });

            let consumer_shared = Arc::clone(&shared);
            let consumer_block = Arc::clone(&block_available);
            let consumer = thread::spawn(move || loop {
                if consumer_shared.generation.load(Ordering::Acquire) != my_gen
                    || consumer_shared.stop_requested.load(Ordering::Relaxed)
                {
                    break;
                }
                if consumer_block.swap(false, Ordering::AcqRel) {
                    continue;
                }
                thread::yield_now();
            });

            shared.stop_requested.store(true, Ordering::SeqCst);
            shared.generation.fetch_add(1, Ordering::AcqRel);
            producer.join().unwrap();
            consumer.join().unwrap();
            shared.streaming.store(false, Ordering::SeqCst);
            shared.stop_requested.store(false, Ordering::SeqCst);

            assert!(!shared.streaming.load(Ordering::SeqCst));
            assert!(!shared.stop_requested.load(Ordering::SeqCst));
            assert_ne!(shared.generation.load(Ordering::Acquire), my_gen);
        });
    }
}

/// Convert one raw interleaved (im, re) i16 block into `output`, applying IQ
/// correction and fine-tuning. Mirrors `convert_samples` in `airspyhf.cpp`.
/// Runs on the consumer thread over its own buffers plus `&StreamShared`.
fn convert_samples(
    shared: &StreamShared,
    output: &mut Vec<AirspyhfComplexFloat>,
    rot: &mut AirspyhfComplexFloat,
    src: &[i16],
) -> usize {
    let count = src.len() / 2;
    if output.len() < count {
        output.resize(count, AirspyhfComplexFloat { re: 0.0, im: 0.0 });
    }
    let conv_gain = (1.0f32 / 32768.0) * shared.filter_gain();
    for (i, o) in output.iter_mut().enumerate().take(count) {
        // Wire order is (im, re) per airspyhf_complex_int16_t.
        let im = src[2 * i] as f32;
        let re = src[2 * i + 1] as f32;
        o.re = re * conv_gain;
        o.im = im * conv_gain;
    }

    if shared.enable_dsp() != 0 {
        if !shared.is_low_if() {
            if let Ok(mut bal) = shared.balancer.lock() {
                bal.process(&mut output[..count]);
            }
        }
        let freq_shift = shared.freq_shift();
        if freq_shift != 0.0 {
            let angle =
                2.0 * std::f64::consts::PI * freq_shift / shared.current_samplerate() as f64;
            let rotc = AirspyhfComplexFloat {
                re: angle.cos() as f32,
                im: -(angle.sin() as f32),
            };
            for s in &mut output[..count] {
                rotate_complex(rot, &rotc);
                multiply_complex_complex(s, rot);
            }
        }
    }
    count
}

fn multiply_complex_complex(a: &mut AirspyhfComplexFloat, b: &AirspyhfComplexFloat) {
    let re = a.re * b.re - a.im * b.im;
    a.im = a.im * b.re + a.re * b.im;
    a.re = re;
}

fn rotate_complex(vec: &mut AirspyhfComplexFloat, rot: &AirspyhfComplexFloat) {
    multiply_complex_complex(vec, rot);
    let norm = 1.99 - (vec.re * vec.re + vec.im * vec.im);
    vec.re *= norm;
    vec.im *= norm;
}

/// USB worker (producer): pull bulk-IN transfers, hand raw blocks to the queue,
/// counting drops when the consumer falls behind. Touches only `Arc<StreamShared>`.
fn producer_proc(
    shared: Arc<StreamShared>,
    my_gen: u32,
    iface: Interface,
    full_tx: SyncSender<RawBlock>,
    empty_rx: Receiver<Vec<i16>>,
) {
    if iface.set_alt_setting(1).wait().is_err() {
        retire_stream_generation(&shared, my_gen);
        return;
    }
    let mut ep = match iface
        .endpoint::<nusb::transfer::Bulk, nusb::transfer::In>(0x80 | AIRSPYHF_ENDPOINT_IN)
    {
        Ok(e) => e,
        Err(_) => {
            retire_stream_generation(&shared, my_gen);
            return;
        }
    };
    let _ = ep.clear_halt().wait();

    let buf_len = (SAMPLES_TO_TRANSFER as usize) * 4; // bytes: 2 * i16 per complex sample
    for _ in 0..RAW_BUFFER_COUNT {
        let mut b = ep.allocate(buf_len);
        b.set_requested_len(buf_len);
        ep.submit(b);
    }

    let mut pending_dropped: u32 = 0;
    loop {
        let comp = ep.wait_next_complete(Duration::from_millis(PRODUCER_WAIT_MS));
        if shared.generation.load(Ordering::Acquire) != my_gen
            || shared.stop_requested.load(Ordering::Relaxed)
        {
            cancel_and_drain_endpoint(&mut ep);
            break;
        }
        let Some(c) = comp else { continue };
        if let Err(e) = c.status {
            if e == nusb::transfer::TransferError::Stall {
                cancel_and_drain_endpoint(&mut ep);
                let _ = ep.clear_halt().wait();
            }
            retire_stream_generation(&shared, my_gen);
            break;
        }

        // Take a recycled buffer if the consumer has kept up; otherwise this
        // block is dropped (queue full) and only counted.
        match empty_rx.try_recv() {
            Ok(mut data) => {
                data.clear();
                // Native-endian i16 decode (all supported hosts are little-endian,
                // matching the C++ reinterpret). No pointer cast, no alignment
                // assumption.
                data.extend(
                    c.buffer
                        .chunks_exact(2)
                        .map(|b| i16::from_ne_bytes([b[0], b[1]])),
                );
                match full_tx.try_send(RawBlock {
                    data,
                    dropped: pending_dropped,
                }) {
                    Ok(()) => pending_dropped = 0,
                    Err(TrySendError::Full(_)) | Err(TrySendError::Disconnected(_)) => {
                        pending_dropped = pending_dropped.saturating_add(1);
                    }
                }
            }
            Err(_) => pending_dropped = pending_dropped.saturating_add(1),
        }
        ep.submit(c.buffer);
    }
}

fn retire_stream_generation(shared: &StreamShared, my_gen: u32) {
    if shared.generation.load(Ordering::Acquire) == my_gen {
        shared.stop_requested.store(true, Ordering::SeqCst);
        shared.streaming.store(false, Ordering::SeqCst);
    }
}

fn cancel_and_drain_endpoint(ep: &mut nusb::Endpoint<nusb::transfer::Bulk, nusb::transfer::In>) {
    if ep.pending() == 0 {
        return;
    }

    ep.cancel_all();
    while ep.pending() > 0 {
        if ep
            .wait_next_complete(Duration::from_millis(PRODUCER_CANCEL_DRAIN_MS))
            .is_none()
        {
            break;
        }
    }
}

/// Sample worker (consumer): convert queued blocks and fire the user callback.
/// Owns its output buffer and rotation state; shares only `Arc<StreamShared>`.
fn consumer_proc(
    shared: Arc<StreamShared>,
    my_gen: u32,
    cbctx: CallbackCtx,
    full_rx: Receiver<RawBlock>,
    empty_tx: SyncSender<Vec<i16>>,
) {
    let mut output: Vec<AirspyhfComplexFloat> = Vec::with_capacity(SAMPLES_TO_TRANSFER as usize);
    let mut rot = AirspyhfComplexFloat { re: 1.0, im: 0.0 };

    loop {
        match full_rx.recv_timeout(Duration::from_millis(200)) {
            Ok(block) => {
                if shared.generation.load(Ordering::Acquire) != my_gen
                    || shared.stop_requested.load(Ordering::Relaxed)
                {
                    break;
                }
                let n = convert_samples(&shared, &mut output, &mut rot, &block.data);
                // Recycle the raw buffer.
                let _ = empty_tx.try_send(block.data);

                if let Some(cb) = cbctx.cb {
                    let mut xfer = AirspyhfTransfer {
                        device: cbctx.dev_ptr as *mut AirspyHfDevice,
                        ctx: cbctx.ctx as *mut std::ffi::c_void,
                        samples: output.as_mut_ptr(),
                        sample_count: n as i32,
                        dropped_samples: block.dropped as u64 * n as u64,
                    };
                    // SAFETY: `cb` is the caller-provided C callback; `xfer` is a
                    // valid, exclusively-owned transfer for the duration of the
                    // call, and `samples` points at `output` which lives past it.
                    let stop = unsafe { cb(&mut xfer as *mut _) } != 0;
                    if stop {
                        shared.stop_requested.store(true, Ordering::SeqCst);
                        shared.streaming.store(false, Ordering::SeqCst);
                        break;
                    }
                }
            }
            Err(RecvTimeoutError::Timeout) => {
                if shared.generation.load(Ordering::Acquire) != my_gen
                    || shared.stop_requested.load(Ordering::Relaxed)
                {
                    break;
                }
            }
            Err(RecvTimeoutError::Disconnected) => break,
        }
    }
    retire_stream_generation(&shared, my_gen);
}
