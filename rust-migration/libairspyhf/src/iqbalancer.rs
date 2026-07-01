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

//! Rust port of `iqbalancer.cpp`: the FFT-based automatic I/Q imbalance
//! correction used by the AirspyHF+ driver in Zero-IF mode.
//!
//! This port is kept in step with the C++ reference for full numerical parity:
//!
//! * Phase / amplitude / DC-removal accumulators are `f64` (the C++ struct uses
//!   `double`; the per-sample values are cast back to `f32` when applied, just
//!   like the C++ `dfloat()` casts).
//! * `rustfft`'s forward transform is followed by a half-swap (fftshift) so the
//!   spectrum is centered on bin `FFTBINS/2`, matching the custom C++ `fft()`
//!   that swaps the two halves at the end.
//! * Correlation loop bounds match the C++ (`i < FFTBins/2 - 1` for the lower
//!   half).

use crate::AirspyhfComplexFloat;
use num_complex::Complex32;
use rustfft::{Fft, FftPlanner};
use std::f32::consts::PI;
use std::panic::{catch_unwind, AssertUnwindSafe};
use std::sync::{Arc, OnceLock};

const FFTBINS: usize = 4 * 1024;
const BOOST_FACTOR: f32 = 100000.0;
const BINS_TO_OPTIMIZE: usize = FFTBINS / 25;
const EDGE_BINS_TO_SKIP: usize = FFTBINS / 22;
const CENTER_BINS_TO_SKIP: usize = 2;
const MAX_LOOKBACK: usize = 4;

// The exact literals from `iqbalancer.h`; the compiler rounds to the nearest
// f32 (same value the C++ build uses).
#[allow(clippy::excessive_precision)]
const PHASE_GRAD_STEP: f32 = 0.414213562373; // tan(pi/2*MaxLookback)
#[allow(clippy::excessive_precision)]
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

// DC-removal smoothing factor; the C++ `iq_balancer_process` uses 1e-4f directly.
const DC_ALPHA: f32 = 1e-4;

const BUFFERS_TO_SKIP_DEFAULT: i32 = 1;
const FFT_INTEGRATION_DEFAULT: i32 = 8;
const FFT_OVERLAP_DEFAULT: i32 = 4;
const CORRELATION_INTEGRATION_DEFAULT: i32 = 32;
const MAX_BUFFERS_TO_SKIP_CONFIG: i32 = 1024;
const MAX_FFT_INTEGRATION_CONFIG: i32 = 64;
const MAX_FFT_OVERLAP_CONFIG: i32 = 64;
const MAX_CORRELATION_INTEGRATION_CONFIG: i32 = 4096;

const WORKING_BUFFER_LENGTH: usize =
    FFTBINS * (1 + (FFT_INTEGRATION_DEFAULT as usize / FFT_OVERLAP_DEFAULT as usize));

#[allow(clippy::excessive_precision)]
const FFT_WINDOW_A0: f32 = 0.35767769;
#[allow(clippy::excessive_precision)]
const FFT_WINDOW_A1: f32 = 0.48784312;
#[allow(clippy::excessive_precision)]
const FFT_WINDOW_A2: f32 = 0.14236325;
#[allow(clippy::excessive_precision)]
const FFT_WINDOW_A3: f32 = 0.01211594;

static FFT_WINDOW: OnceLock<Vec<f32>> = OnceLock::new();
static BOOST_WINDOW: OnceLock<Vec<f32>> = OnceLock::new();
static DISTANCE_WEIGHTS: OnceLock<Vec<f32>> = OnceLock::new();

fn correction_mu(numerator: f32, denominator: f32) -> f32 {
    if denominator.abs() > MIN_DELTA_MU {
        (numerator / denominator).clamp(-MAX_MU, MAX_MU)
    } else {
        0.0
    }
}

fn optimal_point_and_bin(w: f32) -> (f32, usize) {
    let clamped = if w.is_finite() {
        w.clamp(-0.5, 0.5)
    } else {
        0.0
    };
    let bin = ((FFTBINS as f32) * (0.5 + clamped)).floor() as usize;
    (clamped, bin.min(FFTBINS))
}

fn init_library() {
    FFT_WINDOW.get_or_init(|| {
        let length = FFTBINS - 1;
        let mut win = vec![0.0f32; FFTBINS];
        let mut boost = vec![0.0f32; FFTBINS];
        for i in 0..=length {
            win[i] = FFT_WINDOW_A0 - FFT_WINDOW_A1 * (2.0 * PI * i as f32 / length as f32).cos()
                + FFT_WINDOW_A2 * (4.0 * PI * i as f32 / length as f32).cos()
                - FFT_WINDOW_A3 * (6.0 * PI * i as f32 / length as f32).cos();
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
/// Performs full FFT-based imbalance estimation and applies phase and amplitude
/// corrections that mirror the C++ logic.
#[repr(C)]
pub struct IqBalancer {
    // Estimation state kept in f64 to match the C++ `double` accumulators.
    pub phase: f64,
    pub last_phase: f64,
    pub amplitude: f64,
    pub last_amplitude: f64,
    pub iavg: f64,
    pub qavg: f64,
    pub iavg_after: f64,
    pub qavg_after: f64,

    pub optimal_point: f32,
    pub buffers_to_skip: i32,
    pub fft_integration: i32,
    pub fft_overlap: i32,
    pub correlation_integration: i32,
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
    /// Reused FFT frame buffer, so `compute_corr` allocates nothing per call.
    fft_scratch: Vec<Complex32>,
}

impl IqBalancer {
    pub fn new(initial_phase: f32, initial_amplitude: f32) -> Self {
        init_library();
        let fft = FftPlanner::new().plan_fft_forward(FFTBINS);
        Self {
            phase: initial_phase as f64,
            last_phase: initial_phase as f64,
            amplitude: initial_amplitude as f64,
            last_amplitude: initial_amplitude as f64,
            iavg: 0.0,
            qavg: 0.0,
            iavg_after: 0.0,
            qavg_after: 0.0,
            optimal_point: 0.0,
            buffers_to_skip: BUFFERS_TO_SKIP_DEFAULT,
            fft_integration: FFT_INTEGRATION_DEFAULT,
            fft_overlap: FFT_OVERLAP_DEFAULT,
            correlation_integration: CORRELATION_INTEGRATION_DEFAULT,
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
            fft_scratch: vec![Complex32::new(0.0, 0.0); FFTBINS],
        }
    }

    pub fn set_optimal_point(&mut self, w: f32) {
        let (clamped, bin) = optimal_point_and_bin(w);
        self.optimal_point = clamped;
        self.optimal_bin = bin;
        self.reset_flag = true;
    }

    pub fn configure(
        &mut self,
        buffers_to_skip: i32,
        fft_integration: i32,
        fft_overlap: i32,
        correlation_integration: i32,
    ) {
        self.buffers_to_skip = buffers_to_skip.clamp(0, MAX_BUFFERS_TO_SKIP_CONFIG);
        self.fft_integration = fft_integration.clamp(1, MAX_FFT_INTEGRATION_CONFIG);
        self.fft_overlap = fft_overlap.clamp(1, MAX_FFT_OVERLAP_CONFIG);
        self.correlation_integration =
            correlation_integration.clamp(1, MAX_CORRELATION_INTEGRATION_CONFIG);
        self.power_flag.resize(self.fft_integration as usize, 0);
        self.reset_flag = true;
    }

    fn cancel_dc(&mut self, slice: &mut [AirspyhfComplexFloat], alpha: f32) {
        let alpha = alpha as f64;
        let mut iavg = self.iavg;
        let mut qavg = self.qavg;
        for s in slice {
            iavg = (1.0 - alpha) * iavg + alpha * s.re as f64;
            qavg = (1.0 - alpha) * qavg + alpha * s.im as f64;
            s.re -= iavg as f32;
            s.im -= qavg as f32;
        }
        self.iavg = iavg;
        self.qavg = qavg;
    }

    fn cancel_dc_after(&mut self, slice: &mut [AirspyhfComplexFloat], alpha: f32) {
        let alpha = alpha as f64;
        let mut iavg = self.iavg_after;
        let mut qavg = self.qavg_after;
        for s in slice {
            iavg = (1.0 - alpha) * iavg + alpha * s.re as f64;
            qavg = (1.0 - alpha) * qavg + alpha * s.im as f64;
            s.re -= iavg as f32;
            s.im -= qavg as f32;
        }
        self.iavg_after = iavg;
        self.qavg_after = qavg;
    }

    fn adjust_phase_amplitude(&mut self, slice: &mut [AirspyhfComplexFloat]) {
        let len = slice.len();
        if len <= 1 {
            return;
        }
        let scale = (1.0f32 / (len - 1) as f32) as f64;
        for (i, s) in slice.iter_mut().enumerate() {
            let fi = i as f64;
            let fl = (len - 1 - i) as f64;
            let p = (fi * self.last_phase + fl * self.phase) * scale;
            let a = (fi * self.last_amplitude + fl * self.amplitude) * scale;
            let re = s.re;
            let im = s.im;
            s.re += (p * im as f64) as f32;
            s.im += (p * re as f64) as f32;
            s.re = (s.re as f64 * (1.0 + a)) as f32;
            s.im = (s.im as f64 * (1.0 - a)) as f32;
        }
        self.last_phase = self.phase;
        self.last_amplitude = self.amplitude;
    }

    /// FFT of `buf` matching the C++ `fft()`: forward transform followed by a
    /// half-swap so the spectrum is centered on `FFTBINS/2` (fftshift).
    fn fft_shifted(&self, buf: &mut [Complex32]) {
        Self::window(buf);
        self.fft.process(buf);
        let (lo, hi) = buf.split_at_mut(FFTBINS / 2);
        lo.swap_with_slice(hi);
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

    fn adjust_benchmark_return_sum(&mut self, iq: &mut [Complex32], length: usize) -> f32 {
        let mut sum = 0.0f64;
        let scale = (1.0f32 / (length - 1) as f32) as f64;
        for (i, s) in iq.iter_mut().enumerate() {
            let fi = i as f64;
            let fl = (length - 1 - i) as f64;
            let phase = (fi * self.last_phase + fl * self.phase) * scale;
            let amplitude = (fi * self.last_amplitude + fl * self.amplitude) * scale;
            let re = s.re;
            let im = s.im;
            s.re += (phase * im as f64) as f32;
            s.im += (phase * re as f64) as f32;
            s.re = (s.re as f64 * (1.0 + amplitude)) as f32;
            s.im = (s.im as f64 * (1.0 - amplitude)) as f32;
            sum += (re * re + im * im) as f64;
        }
        sum as f32
    }

    // The correlation/boost loops index several arrays by the same running bin
    // index; range-form loops read closest to the C++ reference.
    #[allow(clippy::needless_range_loop)]
    fn compute_corr_inner(
        &mut self,
        iq: &[AirspyhfComplexFloat],
        ccorr: &mut [Complex32],
        length: usize,
        step: i32,
    ) -> i32 {
        let mut count = 0;
        let phase = (self.phase + (step as f32 * PHASE_STEP) as f64) as f32;
        let amplitude = (self.amplitude + (step as f32 * AMPLITUDE_STEP) as f64) as f32;
        let optimal_bin = self.optimal_bin;
        // Reuse the persistent scratch buffer (no per-call allocation). Taken out
        // to satisfy the borrow checker while `&mut self` methods run, restored
        // before returning.
        let mut fft_buf = std::mem::take(&mut self.fft_scratch);
        if fft_buf.len() != FFTBINS {
            fft_buf = vec![Complex32::new(0.0, 0.0); FFTBINS];
        }
        let step_len = FFTBINS / self.fft_overlap as usize;
        let mut m = 0i32;
        let mut n = 0usize;
        while n + FFTBINS <= length && m < self.fft_integration {
            for i in 0..FFTBINS {
                fft_buf[i].re = iq[n + i].re;
                fft_buf[i].im = iq[n + i].im;
            }
            if step == 0 {
                self.power_flag[m as usize] = 0;
                let power = self.adjust_benchmark_return_sum(&mut fft_buf, length);
                if power > MINIMUM_POWER {
                    self.power_flag[m as usize] = 1;
                    self.integrated_total_power += power as f64;
                    count += 1;
                    self.fft_shifted(&mut fft_buf);
                    Self::accumulate_ccorr(&fft_buf, ccorr);
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
                            let idx = (FFTBINS as isize - i as isize - optimal_bin as isize)
                                .unsigned_abs();
                            self.integrated_image_power += p as f64 * boost_win[idx] as f64;
                        }
                    }
                }
            } else {
                Self::adjust_benchmark_no_sum(&mut fft_buf, phase, amplitude);
                if self.power_flag[m as usize] == 1 {
                    self.fft_shifted(&mut fft_buf);
                    Self::accumulate_ccorr(&fft_buf, ccorr);
                }
            }
            n += step_len;
            m += 1;
        }
        self.fft_scratch = fft_buf;
        count
    }

    /// Accumulate the image correlation for one (windowed, shifted) FFT frame.
    /// Loop bounds mirror the C++ `compute_corr` exactly, including the
    /// `< FFTBins/2 - 1` upper limit for the lower half.
    fn accumulate_ccorr(fft_buf: &[Complex32], ccorr: &mut [Complex32]) {
        for i in EDGE_BINS_TO_SKIP..(FFTBINS / 2 - 1) {
            let cc = Self::multiply_complex_complex(fft_buf[i], fft_buf[FFTBINS - i]);
            ccorr[i].re += cc.re;
            ccorr[i].im += cc.im;
        }
        for i in EDGE_BINS_TO_SKIP..(FFTBINS / 2 - 1) {
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

    fn compute_corr(
        &mut self,
        iq: &[AirspyhfComplexFloat],
        use_plus: bool,
        length: usize,
        step: i32,
    ) -> i32 {
        // corr / corr_plus are disjoint from every other field touched here, but
        // both are owned by `self`; take the buffer out to satisfy the borrow
        // checker, then put it back.
        let mut buf = if use_plus {
            std::mem::take(&mut self.corr_plus)
        } else {
            std::mem::take(&mut self.corr)
        };
        let count = self.compute_corr_inner(iq, &mut buf, length, step);
        if use_plus {
            self.corr_plus = buf;
        } else {
            self.corr = buf;
        }
        count
    }

    fn utility(&self, ccorr: &[Complex32]) -> Complex32 {
        let boost_window = BOOST_WINDOW.get().unwrap();
        let distance_weights = DISTANCE_WEIGHTS.get().unwrap();
        let center = FFTBINS / 2;
        let mut acc = Complex32::new(0.0, 0.0);
        let use_bw = self.optimal_bin != center;
        // The two ranges intentionally overlap at `center + CENTER_BINS_TO_SKIP`
        // (that bin is counted twice), matching the C++ `utility`.
        for i in EDGE_BINS_TO_SKIP..=center + CENTER_BINS_TO_SKIP {
            let boost_factor = self.boost[FFTBINS - i] / (self.boost[i] + EPSILON);
            let mut w = distance_weights[i] * boost_factor;
            if use_bw {
                w *= boost_window[(self.optimal_bin as isize - i as isize).unsigned_abs()];
            }
            acc.re += ccorr[i].re * w;
            acc.im += ccorr[i].im * w;
        }
        for i in center + CENTER_BINS_TO_SKIP..=FFTBINS - EDGE_BINS_TO_SKIP {
            let boost_factor = self.boost[FFTBINS - i] / (self.boost[i] + EPSILON);
            let mut w = distance_weights[i] * boost_factor;
            if use_bw {
                w *= boost_window[(self.optimal_bin as isize - i as isize).unsigned_abs()];
            }
            acc.re += ccorr[i].re * w;
            acc.im += ccorr[i].im * w;
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
            self.corr
                .iter_mut()
                .for_each(|c| *c = Complex32::new(0.0, 0.0));
            self.corr_plus
                .iter_mut()
                .for_each(|c| *c = Complex32::new(0.0, 0.0));
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

        let mut mu = correction_mu(a.im, a.im - b.im);
        let mut phase = (self.phase + (PHASE_STEP * mu) as f64) as f32;

        mu = correction_mu(a.re, a.re - b.re);
        let mut amplitude = (self.amplitude + (AMPLITUDE_STEP * mu) as f64) as f32;

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
        self.phase = phase as f64;
        self.amplitude = amplitude as f64;
    }

    pub fn process(&mut self, iq: &mut [AirspyhfComplexFloat]) {
        if iq.is_empty() {
            return;
        }
        self.cancel_dc(iq, DC_ALPHA);

        // Fill the working buffer with a single copy of `min(free, len)` samples,
        // exactly like the C++ `iq_balancer_process`. Any samples in this block
        // beyond what fits are used for output correction below but are not
        // accumulated for estimation (the C++ code drops them the same way).
        let count = (WORKING_BUFFER_LENGTH - self.working_buffer_pos).min(iq.len());
        self.working_buffer[self.working_buffer_pos..self.working_buffer_pos + count]
            .copy_from_slice(&iq[..count]);
        self.working_buffer_pos += count;
        if self.working_buffer_pos >= WORKING_BUFFER_LENGTH {
            self.working_buffer_pos = 0;
            self.skipped_buffers += 1;
            if self.skipped_buffers > self.buffers_to_skip {
                self.skipped_buffers = 0;
                let wb = std::mem::take(&mut self.working_buffer);
                self.estimate_imbalance(&wb, WORKING_BUFFER_LENGTH);
                self.working_buffer = wb;
            }
        }

        self.adjust_phase_amplitude(iq);
        self.cancel_dc_after(iq, DC_ALPHA);
    }
}

#[cfg(test)]
#[allow(clippy::items_after_test_module)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    #[test]
    fn optimal_point_clamps_and_resets() {
        let mut bal = IqBalancer::new(0.0, 0.0);
        bal.reset_flag = false;

        bal.set_optimal_point(10.0);
        assert_eq!(bal.optimal_point, 0.5);
        assert_eq!(bal.optimal_bin, FFTBINS);
        assert!(bal.reset_flag);

        bal.set_optimal_point(f32::NAN);
        assert_eq!(bal.optimal_point, 0.0);
        assert_eq!(bal.optimal_bin, FFTBINS / 2);
    }

    #[test]
    fn process_empty_and_short_buffers_are_finite() {
        let mut bal = IqBalancer::new(0.00006, -0.0045);
        let mut empty = [];
        bal.process(&mut empty);

        let mut samples = vec![
            AirspyhfComplexFloat { re: 0.25, im: -0.5 },
            AirspyhfComplexFloat {
                re: -0.125,
                im: 0.75,
            },
            AirspyhfComplexFloat { re: 0.0, im: 0.0 },
        ];
        bal.process(&mut samples);
        assert!(samples.iter().all(|s| s.re.is_finite() && s.im.is_finite()));
        assert!(bal.phase.is_finite());
        assert!(bal.amplitude.is_finite());
    }

    #[test]
    fn configure_clamps_hostile_inputs() {
        let mut bal = IqBalancer::new(0.00006, -0.0045);
        bal.configure(i32::MIN, 0, 0, i32::MIN);
        assert_eq!(bal.buffers_to_skip, 0);
        assert_eq!(bal.fft_integration, 1);
        assert_eq!(bal.fft_overlap, 1);
        assert_eq!(bal.correlation_integration, 1);
        assert_eq!(bal.power_flag.len(), 1);

        bal.configure(i32::MAX, i32::MAX, i32::MAX, i32::MAX);
        assert_eq!(bal.buffers_to_skip, MAX_BUFFERS_TO_SKIP_CONFIG);
        assert_eq!(bal.fft_integration, MAX_FFT_INTEGRATION_CONFIG);
        assert_eq!(bal.fft_overlap, MAX_FFT_OVERLAP_CONFIG);
        assert_eq!(
            bal.correlation_integration,
            MAX_CORRELATION_INTEGRATION_CONFIG
        );
        assert_eq!(bal.power_flag.len(), MAX_FFT_INTEGRATION_CONFIG as usize);
    }

    proptest! {
        #[test]
        #[cfg_attr(miri, ignore)]
        fn correction_mu_is_bounded(
            numerator in -1.0e6f32..1.0e6,
            denominator in -1.0e6f32..1.0e6,
        ) {
            let mu = correction_mu(numerator, denominator);
            prop_assert!(mu.is_finite());
            prop_assert!((-MAX_MU..=MAX_MU).contains(&mu));
            if denominator.abs() <= MIN_DELTA_MU {
                prop_assert_eq!(mu, 0.0);
            }
        }

        #[test]
        #[cfg_attr(miri, ignore)]
        fn optimal_point_math_stays_in_bounds(w in any::<f32>()) {
            let (point, bin) = optimal_point_and_bin(w);
            prop_assert!(point.is_finite());
            prop_assert!((-0.5..=0.5).contains(&point));
            prop_assert!(bin <= FFTBINS);
        }

        #[test]
        #[cfg_attr(miri, ignore)]
        fn dsp_process_preserves_finite_outputs(
            values in proptest::collection::vec((-1.0f32..1.0, -1.0f32..1.0), 1..128)
        ) {
            let mut bal = IqBalancer::new(0.00006, -0.0045);
            let mut samples: Vec<_> = values
                .into_iter()
                .map(|(re, im)| AirspyhfComplexFloat { re, im })
                .collect();
            bal.process(&mut samples);
            prop_assert!(samples.iter().all(|s| s.re.is_finite() && s.im.is_finite()));
            prop_assert!(bal.phase.is_finite());
            prop_assert!(bal.amplitude.is_finite());
        }
    }
}

#[cfg(kani)]
mod kani_proofs {
    use super::*;

    #[kani::proof]
    fn optimal_point_is_clamped_and_bin_is_bounded() {
        let w: f32 = kani::any();
        let (point, bin) = optimal_point_and_bin(w);
        assert!(point.is_finite());
        assert!((-0.5..=0.5).contains(&point));
        assert!(bin <= FFTBINS);
    }

    #[kani::proof]
    fn mu_correction_is_bounded() {
        let numerator: f32 = kani::any();
        let denominator: f32 = kani::any();
        kani::assume(numerator.is_finite());
        kani::assume(denominator.is_finite());
        let mu = correction_mu(numerator, denominator);
        assert!(mu.is_finite());
        assert!((-MAX_MU..=MAX_MU).contains(&mu));
    }
}

// ---------------------------------------------------------------------------
// C ABI mirroring `iqbalancer.h`.
// ---------------------------------------------------------------------------

/// # Safety
/// The returned pointer must be released with [`iq_balancer_destroy`].
#[no_mangle]
pub unsafe extern "C" fn iq_balancer_create(
    initial_phase: f32,
    initial_amplitude: f32,
) -> *mut IqBalancer {
    // Panic guard: never unwind across the C ABI (return null on failure).
    catch_unwind(AssertUnwindSafe(|| {
        Box::into_raw(Box::new(IqBalancer::new(initial_phase, initial_amplitude)))
    }))
    .unwrap_or(std::ptr::null_mut())
}

/// # Safety
/// `ptr` must have come from [`iq_balancer_create`] and not been freed already.
#[no_mangle]
pub unsafe extern "C" fn iq_balancer_destroy(ptr: *mut IqBalancer) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        if !ptr.is_null() {
            // SAFETY: `ptr` came from `iq_balancer_create` (Box::into_raw) and the
            // caller guarantees it is not freed twice or aliased.
            drop(unsafe { Box::from_raw(ptr) });
        }
    }));
}

/// # Safety
/// `ptr` must be a valid balancer pointer.
#[no_mangle]
pub unsafe extern "C" fn iq_balancer_set_optimal_point(ptr: *mut IqBalancer, w: f32) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: caller guarantees `ptr` is a valid balancer or null.
        if let Some(bal) = unsafe { ptr.as_mut() } {
            bal.set_optimal_point(w);
        }
    }));
}

/// # Safety
/// `ptr` must be a valid balancer pointer.
#[no_mangle]
pub unsafe extern "C" fn iq_balancer_configure(
    ptr: *mut IqBalancer,
    buffers_to_skip: i32,
    fft_integration: i32,
    fft_overlap: i32,
    correlation_integration: i32,
) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: caller guarantees `ptr` is a valid balancer or null.
        if let Some(bal) = unsafe { ptr.as_mut() } {
            bal.configure(
                buffers_to_skip,
                fft_integration,
                fft_overlap,
                correlation_integration,
            );
        }
    }));
}

/// # Safety
/// `ptr` must be a valid balancer pointer and `iq` must point to `length`
/// [`AirspyhfComplexFloat`] samples.
#[no_mangle]
pub unsafe extern "C" fn iq_balancer_process(
    ptr: *mut IqBalancer,
    iq: *mut AirspyhfComplexFloat,
    length: i32,
) {
    let _ = catch_unwind(AssertUnwindSafe(|| {
        // SAFETY: caller guarantees `ptr` is a valid balancer or null.
        if let Some(bal) = unsafe { ptr.as_mut() } {
            if !iq.is_null() && length > 0 {
                // SAFETY: caller guarantees `iq` points to `length` samples.
                let slice = unsafe { std::slice::from_raw_parts_mut(iq, length as usize) };
                bal.process(slice);
            }
        }
    }));
}
