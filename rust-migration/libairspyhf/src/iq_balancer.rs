use crate::AirspyhfComplexFloat;

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
}

impl IqBalancer {
    pub fn new(initial_phase: f32, initial_amplitude: f32) -> Self {
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
            working_buffer: Vec::new(),
            skipped_buffers: 0,
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

    fn estimate_imbalance(&mut self, buf: &[AirspyhfComplexFloat]) {
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
            let target = 4096; // placeholder for FFTBins
            if self.working_buffer.len() >= target {
                if self.skipped_buffers >= self.buffers_to_skip {
                    let buf = self.working_buffer.clone();
                    self.estimate_imbalance(&buf);
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
