/*
Copyright (c) 2016-2023, Youssef Touil <youssef@airspy.com>
Copyright (c) 2018, Leif Asbrink <leif@sm5bsz.com>
Copyright (C) 2024, Joshuah Rainstar <joshuah.rainstar@gmail.com>
Contributions to this work were provided by OpenAI Codex, an artificial general intelligence.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
    * Neither the name of Airspy HF+ nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

use crate::AirspyhfComplexFloat;
use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};
use std::f32::consts::PI;
use std::sync::{Arc, OnceLock};

const FFTBINS: usize = 4 * 1024;
const BOOST_FACTOR: f32 = 100000.0;
const BINS_TO_OPTIMIZE: usize = FFTBINS / 25;
const EDGE_BINS_TO_SKIP: usize = FFTBINS / 22;
const CENTER_BINS_TO_SKIP: usize = 2;
const MAX_LOOKBACK: usize = 4;

const PHASE_GRAD_STEP: f32 = 0.414213562373; // tan(pi/2*MaxLookback)
const AMPLITUDE_GRAD_STEP: f32 = 0.9636633660515027;
const PHASE_STEP: f32 = AMPLITUDE_GRAD_STEP;
const AMPLITUDE_STEP: f32 = PHASE_GRAD_STEP;
const MAX_MU: f32 = 50.0;
const MIN_DELTA_MU: f32 = 1e-5;
const MINIMUM_POWER: f32 = 1e-5;
const POWER_THRESHOLD: f32 = 0.5;
const BUFFERS_TO_SKIP_ON_RESET: i32 = 2;
const MAX_POWER_DECAY: f32 = 0.98;
const MAX_POWER_RATIO: f32 = 0.8;
const BOOST_WINDOW_NORM: f32 = MAX_POWER_RATIO / 95.0;
const EPSILON: f32 = 0.00000762939;

const BUFFERS_TO_SKIP_DEFAULT: i32 = 1;
const FFT_INTEGRATION_DEFAULT: i32 = 8;
const FFT_OVERLAP_DEFAULT: i32 = 4;
const CORRELATION_INTEGRATION_DEFAULT: i32 = 32;

const WORKING_BUFFER_LENGTH: usize =
    FFTBINS * (1 + (FFT_INTEGRATION_DEFAULT as usize / FFT_OVERLAP_DEFAULT as usize));

static FFT_WINDOW: OnceLock<Vec<f32>> = OnceLock::new();
static BOOST_WINDOW: OnceLock<Vec<f32>> = OnceLock::new();
static DISTANCE_WEIGHTS: OnceLock<Vec<f32>> = OnceLock::new();

fn init_library() {
    FFT_WINDOW.get_or_init(|| {
        let length = FFTBINS - 1;
        let mut win = vec![0.0f32; FFTBINS];
        let mut boost = vec![0.0f32; FFTBINS];
        for i in 0..=length {
            win[i] = 0.35875 - 0.48829 * (2.0 * PI * i as f32 / length as f32).cos()
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

/// Port of the complex IQ balancer used in the C++ driver.
///
/// This implementation performs the full FFT based imbalance estimation
/// and applies phase and amplitude corrections that mirror the C++ logic.
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
    pub working_buffer_pos: usize,
    pub skipped_buffers: i32,
    pub fft: Arc<dyn Fft<f32>>,
    pub integrated_total_power: f64,
    pub integrated_image_power: f64,
    pub maximum_image_power: f64,
    pub raw_phases: [f32; MAX_LOOKBACK],
    pub raw_amplitudes: [f32; MAX_LOOKBACK],
    pub no_of_avg: i32,
    pub no_of_raw: i32,
    pub raw_ptr: usize,
    pub optimal_bin: usize,
    pub reset_flag: bool,
    pub power_flag: Vec<i32>,
    pub corr: Vec<Complex32>,
    pub corr_plus: Vec<Complex32>,
    pub boost: Vec<f32>,
    pub alpha: f32,
}

impl IqBalancer {
    pub fn new(initial_phase: f32, initial_amplitude: f32) -> Self {
        init_library();
        let fft = FftPlanner::new().plan_fft_forward(FFTBINS);
        Self {
            phase: initial_phase,
            last_phase: initial_phase,
            amplitude: initial_amplitude,
            last_amplitude: initial_amplitude,
            optimal_point: 0.0,
            buffers_to_skip: BUFFERS_TO_SKIP_DEFAULT,
            fft_integration: FFT_INTEGRATION_DEFAULT,
            fft_overlap: FFT_OVERLAP_DEFAULT,
            correlation_integration: CORRELATION_INTEGRATION_DEFAULT,
            iavg: 0.0,
            qavg: 0.0,
            iavg_after: 0.0,
            qavg_after: 0.0,
            working_buffer: vec![AirspyhfComplexFloat { re: 0.0, im: 0.0 }; WORKING_BUFFER_LENGTH],
            working_buffer_pos: 0,
            skipped_buffers: 0,
            fft,
            integrated_total_power: 0.0,
            integrated_image_power: 0.0,
            maximum_image_power: 0.0,
            raw_phases: [0.0; MAX_LOOKBACK],
            raw_amplitudes: [0.0; MAX_LOOKBACK],
            no_of_avg: 0,
            no_of_raw: 0,
            raw_ptr: 0,
            optimal_bin: FFTBINS / 2,
            reset_flag: true,
            power_flag: vec![0; FFT_INTEGRATION_DEFAULT as usize],
            corr: vec![Complex32::new(0.0, 0.0); FFTBINS],
            corr_plus: vec![Complex32::new(0.0, 0.0); FFTBINS],
            boost: vec![0.0; FFTBINS],
            alpha: 1e-4,
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
        self.optimal_bin = ((FFTBINS as f32) * (0.5 + clamped)).floor() as usize;
        self.reset_flag = true;
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
        self.power_flag.resize(fft_integration as usize, 0);
        self.reset_flag = true;
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
            let a =
                (i as f32 * self.last_amplitude + (len - 1.0 - i as f32) * self.amplitude) * scale;
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

    fn window(buf: &mut [Complex32]) {
        let win = FFT_WINDOW.get().expect("window not initialized");
        for (s, w) in buf.iter_mut().zip(win.iter()) {
            s.re *= *w;
            s.im *= *w;
        }
    }

    fn multiply_complex_complex(a: Complex32, b: Complex32) -> Complex32 {
        Complex32::new(a.re * b.re - a.im * b.im, a.im * b.re + a.re * b.im)
    }

    fn adjust_benchmark_no_sum(iq: &mut [Complex32], phase: f32, amplitude: f32) {
        for s in iq.iter_mut() {
            let re = s.re;
            let im = s.im;
            s.re += phase * im;
            s.im += phase * re;
            s.re *= 1.0 + amplitude;
            s.im *= 1.0 - amplitude;
        }
    }

    fn adjust_benchmark_return_sum(
        &mut self,
        iq: &mut [Complex32],
        _p: f32,
        _a: f32,
        length: usize,
    ) -> f32 {
        let mut sum = 0.0f32;
        let scale = 1.0 / (length as f32 - 1.0);
        for (i, s) in iq.iter_mut().enumerate() {
            let phase = (i as f32 * self.last_phase
                + (length as f32 - 1.0 - i as f32) * self.phase)
                * scale;
            let amplitude = (i as f32 * self.last_amplitude
                + (length as f32 - 1.0 - i as f32) * self.amplitude)
                * scale;
            let re = s.re;
            let im = s.im;
            s.re += phase * im;
            s.im += phase * re;
            s.re *= 1.0 + amplitude;
            s.im *= 1.0 - amplitude;
            sum += re * re + im * im;
        }
        sum
    }

    fn compute_corr_inner(
        &mut self,
        iq: &[AirspyhfComplexFloat],
        ccorr: &mut [Complex32],
        length: usize,
        step: i32,
    ) -> i32 {
        let mut count = 0;
        let phase = self.phase + step as f32 * PHASE_STEP;
        let amplitude = self.amplitude + step as f32 * AMPLITUDE_STEP;
        let optimal_bin = self.optimal_bin;
        let mut fft_buf = vec![Complex32::new(0.0, 0.0); FFTBINS];
        if step == 0 {
            let mut m = 0;
            let mut n = 0;
            while n + FFTBINS <= length && m < self.fft_integration {
                self.power_flag[m as usize] = 0;
                for i in 0..FFTBINS {
                    fft_buf[i].re = iq[n + i].re;
                    fft_buf[i].im = iq[n + i].im;
                }
                let power =
                    self.adjust_benchmark_return_sum(&mut fft_buf, phase, amplitude, length);
                if power > MINIMUM_POWER {
                    self.power_flag[m as usize] = 1;
                    self.integrated_total_power += power as f64;
                    count += 1;
                    Self::window(&mut fft_buf);
                    self.fft.process(&mut fft_buf);
                    for i in EDGE_BINS_TO_SKIP..FFTBINS / 2 {
                        let cc = Self::multiply_complex_complex(fft_buf[i], fft_buf[FFTBINS - i]);
                        ccorr[i].re += cc.re;
                        ccorr[i].im += cc.im;
                    }
                    for i in EDGE_BINS_TO_SKIP..FFTBINS / 2 {
                        ccorr[FFTBINS - i] = ccorr[i];
                    }
                    for i in FFTBINS / 2..=FFTBINS - EDGE_BINS_TO_SKIP {
                        let cc = Self::multiply_complex_complex(fft_buf[i], fft_buf[FFTBINS - i]);
                        ccorr[i].re += cc.re;
                        ccorr[i].im += cc.im;
                    }
                    for i in FFTBINS / 2..=FFTBINS - EDGE_BINS_TO_SKIP {
                        ccorr[FFTBINS - i] = ccorr[i];
                    }
                    if optimal_bin == FFTBINS / 2 {
                        for i in EDGE_BINS_TO_SKIP..=FFTBINS - EDGE_BINS_TO_SKIP {
                            let p = fft_buf[i].re * fft_buf[i].re + fft_buf[i].im * fft_buf[i].im;
                            self.boost[i] += p;
                            self.integrated_image_power += p as f64;
                        }
                    } else {
                        let boost_win = BOOST_WINDOW.get().unwrap();
                        for i in EDGE_BINS_TO_SKIP..=FFTBINS - EDGE_BINS_TO_SKIP {
                            let p = fft_buf[i].re * fft_buf[i].re + fft_buf[i].im * fft_buf[i].im;
                            self.boost[i] += p;
                            let idx = (FFTBINS as isize - i as isize - optimal_bin as isize).abs()
                                as usize;
                            self.integrated_image_power += p as f64 * boost_win[idx] as f64;
                        }
                    }
                }
                n += FFTBINS / self.fft_overlap as usize;
                m += 1;
            }
        } else {
            let mut m = 0;
            let mut n = 0;
            while n + FFTBINS <= length && m < self.fft_integration {
                for i in 0..FFTBINS {
                    fft_buf[i].re = iq[n + i].re;
                    fft_buf[i].im = iq[n + i].im;
                }
                Self::adjust_benchmark_no_sum(&mut fft_buf, phase, amplitude);
                if self.power_flag[m as usize] == 1 {
                    Self::window(&mut fft_buf);
                    self.fft.process(&mut fft_buf);
                    for i in EDGE_BINS_TO_SKIP..FFTBINS / 2 {
                        let cc = Self::multiply_complex_complex(fft_buf[i], fft_buf[FFTBINS - i]);
                        ccorr[i].re += cc.re;
                        ccorr[i].im += cc.im;
                    }
                    for i in EDGE_BINS_TO_SKIP..FFTBINS / 2 {
                        ccorr[FFTBINS - i] = ccorr[i];
                    }
                    for i in FFTBINS / 2..=FFTBINS - EDGE_BINS_TO_SKIP {
                        let cc = Self::multiply_complex_complex(fft_buf[i], fft_buf[FFTBINS - i]);
                        ccorr[i].re += cc.re;
                        ccorr[i].im += cc.im;
                    }
                    for i in FFTBINS / 2..=FFTBINS - EDGE_BINS_TO_SKIP {
                        ccorr[FFTBINS - i] = ccorr[i];
                    }
                }
                n += FFTBINS / self.fft_overlap as usize;
                m += 1;
            }
        }
        count
    }

    fn compute_corr(
        &mut self,
        iq: &[AirspyhfComplexFloat],
        use_plus: bool,
        length: usize,
        step: i32,
    ) -> i32 {
        let ptr;
        let len;
        if use_plus {
            ptr = self.corr_plus.as_mut_ptr();
            len = self.corr_plus.len();
        } else {
            ptr = self.corr.as_mut_ptr();
            len = self.corr.len();
        }
        unsafe {
            let slice = std::slice::from_raw_parts_mut(ptr, len);
            self.compute_corr_inner(iq, slice, length, step)
        }
    }

    fn utility(&self, ccorr: &[Complex32]) -> Complex32 {
        let boost_window = BOOST_WINDOW.get().unwrap();
        let distance_weights = DISTANCE_WEIGHTS.get().unwrap();
        let center = FFTBINS / 2;
        let mut acc = Complex32::new(0.0, 0.0);
        if self.optimal_bin != center {
            for i in EDGE_BINS_TO_SKIP..=center + CENTER_BINS_TO_SKIP {
                let boost_factor = self.boost[FFTBINS - i] / (self.boost[i] + EPSILON);
                let w = distance_weights[i]
                    * boost_factor
                    * boost_window[(self.optimal_bin as isize - i as isize).abs() as usize];
                acc.re += ccorr[i].re * w;
                acc.im += ccorr[i].im * w;
            }
            for i in center + CENTER_BINS_TO_SKIP..=FFTBINS - EDGE_BINS_TO_SKIP {
                let boost_factor = self.boost[FFTBINS - i] / (self.boost[i] + EPSILON);
                let w = distance_weights[i]
                    * boost_factor
                    * boost_window[(self.optimal_bin as isize - i as isize).abs() as usize];
                acc.re += ccorr[i].re * w;
                acc.im += ccorr[i].im * w;
            }
        } else {
            for i in EDGE_BINS_TO_SKIP..=center + CENTER_BINS_TO_SKIP {
                let boost_factor = self.boost[FFTBINS - i] / (self.boost[i] + EPSILON);
                let w = distance_weights[i] * boost_factor;
                acc.re += ccorr[i].re * w;
                acc.im += ccorr[i].im * w;
            }
            for i in center + CENTER_BINS_TO_SKIP..=FFTBINS - EDGE_BINS_TO_SKIP {
                let boost_factor = self.boost[FFTBINS - i] / (self.boost[i] + EPSILON);
                let w = distance_weights[i] * boost_factor;
                acc.re += ccorr[i].re * w;
                acc.im += ccorr[i].im * w;
            }
        }
        acc
    }

    fn estimate_imbalance(&mut self, iq: &[AirspyhfComplexFloat], length: usize) {
        if self.reset_flag {
            self.reset_flag = false;
            self.no_of_avg = -BUFFERS_TO_SKIP_ON_RESET;
            self.maximum_image_power = 0.0;
        }
        if self.no_of_avg < 0 {
            self.no_of_avg += 1;
            return;
        } else if self.no_of_avg == 0 {
            self.integrated_image_power = 0.0;
            self.integrated_total_power = 0.0;
            self.boost.fill(0.0);
            for c in &mut self.corr {
                *c = Complex32::new(0.0, 0.0);
            }
            for c in &mut self.corr_plus {
                *c = Complex32::new(0.0, 0.0);
            }
        }

        self.maximum_image_power *= MAX_POWER_DECAY as f64;

        let i = self.compute_corr(iq, false, length, 0);
        if i == 0 {
            return;
        }

        self.no_of_avg += i;
        self.compute_corr(iq, true, length, 1);

        if self.no_of_avg <= self.correlation_integration * self.fft_integration {
            return;
        }
        self.no_of_avg = 0;

        if self.optimal_bin == FFTBINS / 2 {
            if self.integrated_total_power < self.maximum_image_power {
                return;
            }
            self.maximum_image_power = self.integrated_total_power;
        } else if (self.integrated_image_power
            - self.integrated_total_power * (BOOST_WINDOW_NORM as f64))
            < self.maximum_image_power * (POWER_THRESHOLD as f64)
        {
            return;
        } else {
            self.maximum_image_power = self.integrated_image_power
                - self.integrated_total_power * (BOOST_WINDOW_NORM as f64);
        }

        let a = self.utility(&self.corr);
        let b = self.utility(&self.corr_plus);

        let mut mu = a.im - b.im;
        if mu.abs() > MIN_DELTA_MU {
            mu = a.im / mu;
            if mu < -MAX_MU {
                mu = -MAX_MU;
            } else if mu > MAX_MU {
                mu = MAX_MU;
            }
        } else {
            mu = 0.0;
        }
        let mut phase = self.phase + PHASE_STEP * mu;

        mu = a.re - b.re;
        if mu.abs() > MIN_DELTA_MU {
            mu = a.re / mu;
            if mu < -MAX_MU {
                mu = -MAX_MU;
            } else if mu > MAX_MU {
                mu = MAX_MU;
            }
        } else {
            mu = 0.0;
        }
        let mut amplitude = self.amplitude + AMPLITUDE_STEP * mu;

        if self.no_of_raw < MAX_LOOKBACK as i32 {
            self.no_of_raw += 1;
        }
        self.raw_amplitudes[self.raw_ptr] = amplitude;
        self.raw_phases[self.raw_ptr] = phase;
        let mut idx = self.raw_ptr;
        for _ in 0..(self.no_of_raw - 1) {
            idx = (idx + MAX_LOOKBACK - 1) % MAX_LOOKBACK;
            phase += self.raw_phases[idx];
            amplitude += self.raw_amplitudes[idx];
        }
        phase /= self.no_of_raw as f32;
        amplitude /= self.no_of_raw as f32;
        self.raw_ptr = (self.raw_ptr + 1) % MAX_LOOKBACK;
        self.phase = phase;
        self.amplitude = amplitude;
    }

    pub fn process(&mut self, iq: *mut AirspyhfComplexFloat, length: i32) {
        if iq.is_null() || length <= 0 {
            return;
        }
        unsafe {
            const ALPHA: f32 = 1e-4;
            let slice = std::slice::from_raw_parts_mut(iq, length as usize);
            self.cancel_dc(slice, ALPHA);

            let mut remaining = slice.len();
            let mut offset = 0;
            while remaining > 0 {
                let avail = WORKING_BUFFER_LENGTH - self.working_buffer_pos;
                let count = avail.min(remaining);
                self.working_buffer[self.working_buffer_pos..self.working_buffer_pos + count]
                    .copy_from_slice(&slice[offset..offset + count]);
                self.working_buffer_pos += count;
                offset += count;
                remaining -= count;

                if self.working_buffer_pos >= WORKING_BUFFER_LENGTH {
                    self.working_buffer_pos = 0;
                    if self.skipped_buffers >= self.buffers_to_skip {
                        self.skipped_buffers = 0;
                        let ptr = self.working_buffer.as_ptr();
                        let slice = std::slice::from_raw_parts(ptr, WORKING_BUFFER_LENGTH);
                        self.estimate_imbalance(slice, WORKING_BUFFER_LENGTH);
                    } else {
                        self.skipped_buffers += 1;
                    }
                }
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
