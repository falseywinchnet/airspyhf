use crate::AirspyhfComplexFloat;
use num_complex::Complex32;
use rustfft::{FftPlanner, Fft};
use std::sync::{Arc, OnceLock};
use std::f32::consts::PI;

const FFTBINS: usize = 4 * 1024;
const BOOST_FACTOR: f32 = 100000.0;
const BINS_TO_OPTIMIZE: usize = FFTBINS / 25;
const EDGE_BINS_TO_SKIP: usize = FFTBINS / 22;
const CENTER_BINS_TO_SKIP: usize = 2;
const MAX_LOOKBACK: usize = 4;
const WORKING_BUFFER_LENGTH: usize = FFTBINS;

static FFT_WINDOW: OnceLock<Vec<f32>> = OnceLock::new();
static BOOST_WINDOW: OnceLock<Vec<f32>> = OnceLock::new();
static DISTANCE_WEIGHTS: OnceLock<Vec<f32>> = OnceLock::new();

fn init_library() {
    FFT_WINDOW.get_or_init(|| {
        let length = FFTBINS - 1;
        let mut win = vec![0.0f32; FFTBINS];
        let mut boost = vec![0.0f32; FFTBINS];
        for i in 0..=length {
            win[i] = 0.35875
                - 0.48829 * (2.0 * PI * i as f32 / length as f32).cos()
                + 0.14128 * (4.0 * PI * i as f32 / length as f32).cos()
                - 0.01168 * (6.0 * PI * i as f32 / length as f32).cos();
            boost[i] = 1.0 / BOOST_FACTOR
                + 1.0 / f32::exp((i as f32 * 2.0 / BINS_TO_OPTIMIZE as f32).powi(2));
        }
        BOOST_WINDOW.set(boost).ok();
        let mut dist = vec![1.0f32; FFTBINS];
        let center = FFTBINS / 2;
        let invskip = 1.0 / EDGE_BINS_TO_SKIP as f32;
        for i in (center - EDGE_BINS_TO_SKIP)..=(center - CENTER_BINS_TO_SKIP) {
            let distance = (i as isize - center as isize).abs() as f32;
            let weight = distance * invskip;
            dist[i] = weight;
            dist[FFTBINS - i] = weight;
        }
        DISTANCE_WEIGHTS.set(dist).ok();
        win
    });
}

/// Simplified port of the complex IQ balancer used in the C driver.
///
/// This implementation still omits the heavy FFT based imbalance estimation
/// but follows the original structure so the remaining pieces can be filled in
/// incrementally.  The DC removal and phase/amplitude adjustment mirror the
/// C++ logic.
#[repr(C)]
pub struct IqBalancer {
    pub phase: f32,
    pub last_phase: f32,
    pub amplitude: f32,
    pub last_amplitude: f32,
    pub optimal_point: f32,
    pub buffers_to_skip: i32,
    pub fft_integration: i32,
    pub fft_overlap: i32,
    pub correlation_integration: i32,
    pub iavg: f32,
    pub qavg: f32,
    pub iavg_after: f32,
    pub qavg_after: f32,
    pub working_buffer: Vec<AirspyhfComplexFloat>,
    pub skipped_buffers: i32,
    pub fft: std::sync::Arc<dyn Fft<f32>>,        
}

impl IqBalancer {
    pub fn new(initial_phase: f32, initial_amplitude: f32) -> Self {
        init_library();
        Self {
            phase: initial_phase,
            last_phase: initial_phase,
            amplitude: initial_amplitude,
            last_amplitude: initial_amplitude,
            optimal_point: 0.0,
            buffers_to_skip: 1,
            fft_integration: 8,
            fft_overlap: 4,
            correlation_integration: 32,
            iavg: 0.0,
            qavg: 0.0,
            iavg_after: 0.0,
            qavg_after: 0.0,
            working_buffer: Vec::with_capacity(WORKING_BUFFER_LENGTH),
            skipped_buffers: 0,
            fft: FftPlanner::new().plan_fft_forward(FFTBINS),
        }
    }

    pub fn set_optimal_point(&mut self, w: f32) {
        let mut clamped = w;
        if clamped < -0.5 {
            clamped = -0.5;
        } else if clamped > 0.5 {
            clamped = 0.5;
        }
        self.optimal_point = clamped;
    }

    pub fn configure(
        &mut self,
        buffers_to_skip: i32,
        fft_integration: i32,
        fft_overlap: i32,
        correlation_integration: i32,
    ) {
        self.buffers_to_skip = buffers_to_skip;
        self.fft_integration = fft_integration;
        self.fft_overlap = fft_overlap;
        self.correlation_integration = correlation_integration;
    }

    fn cancel_dc(&mut self, slice: &mut [AirspyhfComplexFloat], alpha: f32) {
        for s in slice {
            self.iavg = (1.0 - alpha) * self.iavg + alpha * s.re;
            self.qavg = (1.0 - alpha) * self.qavg + alpha * s.im;
            s.re -= self.iavg;
            s.im -= self.qavg;
        }
    }

    fn cancel_dc_after(&mut self, slice: &mut [AirspyhfComplexFloat], alpha: f32) {
        for s in slice {
            self.iavg_after = (1.0 - alpha) * self.iavg_after + alpha * s.re;
            self.qavg_after = (1.0 - alpha) * self.qavg_after + alpha * s.im;
            s.re -= self.iavg_after;
            s.im -= self.qavg_after;
        }
    }

    fn adjust_phase_amplitude(&mut self, slice: &mut [AirspyhfComplexFloat]) {
        let len = slice.len() as f32;
        if len <= 1.0 {
            return;
        }
        let scale = 1.0 / (len - 1.0);
        for (i, s) in slice.iter_mut().enumerate() {
            let p = (i as f32 * self.last_phase + (len - 1.0 - i as f32) * self.phase) * scale;
            let a = (i as f32 * self.last_amplitude + (len - 1.0 - i as f32) * self.amplitude) * scale;
            let re = s.re;
            let im = s.im;
            s.re += p * im;
            s.im += p * re;
            s.re *= 1.0 + a;
            s.im *= 1.0 - a;
        }
        self.last_phase = self.phase;
        self.last_amplitude = self.amplitude;
    }

    fn estimate_imbalance(&mut self, buf: &[AirspyhfComplexFloat], fft: &Arc<dyn Fft<f32>>) {
        // Placeholder for the heavy FFT based algorithm.  For now we compute a
        // very rough phase/amplitude estimate from the mean values.
        if buf.is_empty() {
            return;
        }
        let mut i_sum = 0.0f32;
        let mut q_sum = 0.0f32;
        for s in buf {
            i_sum += s.re;
            q_sum += s.im;
        }
        let len = buf.len() as f32;
        let phase_err = (q_sum / len) * 1e-6;
        let amp_err = (i_sum / len) * 1e-6;
        self.phase += phase_err;
        self.amplitude += amp_err;
    }

    pub fn process(&mut self, iq: *mut AirspyhfComplexFloat, length: i32) {
        if iq.is_null() || length <= 0 {
            return;
        }
        unsafe {
            const ALPHA: f32 = 1e-4;
            let slice = std::slice::from_raw_parts_mut(iq, length as usize);
            self.cancel_dc(slice, ALPHA);

            self.working_buffer.extend_from_slice(slice);
            let target = WORKING_BUFFER_LENGTH;
            if self.working_buffer.len() >= target {
                if self.skipped_buffers >= self.buffers_to_skip {
                    let buf = self.working_buffer.clone();
                    let fft = self.fft.clone();
                    self.estimate_imbalance(&buf, &fft);
                    self.skipped_buffers = 0;
                } else {
                    self.skipped_buffers += 1;
                }
                self.working_buffer.clear();
            }

            self.adjust_phase_amplitude(slice);
            self.cancel_dc_after(slice, ALPHA);
        }
    }
}

#[no_mangle]
pub unsafe extern "C" fn iq_balancer_create(
    initial_phase: f32,
    initial_amplitude: f32,
) -> *mut IqBalancer {
    Box::into_raw(Box::new(IqBalancer::new(initial_phase, initial_amplitude)))
}

#[no_mangle]
pub unsafe extern "C" fn iq_balancer_destroy(ptr: *mut IqBalancer) {
    if !ptr.is_null() {
        drop(Box::from_raw(ptr));
    }
}

#[no_mangle]
pub unsafe extern "C" fn iq_balancer_set_optimal_point(ptr: *mut IqBalancer, w: f32) {
    if let Some(bal) = ptr.as_mut() {
        bal.set_optimal_point(w);
    }
}

#[no_mangle]
pub unsafe extern "C" fn iq_balancer_configure(
    ptr: *mut IqBalancer,
    buffers_to_skip: i32,
    fft_integration: i32,
    fft_overlap: i32,
    correlation_integration: i32,
) {
    if let Some(bal) = ptr.as_mut() {
        bal.configure(
            buffers_to_skip,
            fft_integration,
            fft_overlap,
            correlation_integration,
        );
    }
}

#[no_mangle]
pub unsafe extern "C" fn iq_balancer_process(
    ptr: *mut IqBalancer,
    iq: *mut AirspyhfComplexFloat,
    length: i32,
) {
    if let Some(bal) = ptr.as_mut() {
        bal.process(iq, length);
    }
}
