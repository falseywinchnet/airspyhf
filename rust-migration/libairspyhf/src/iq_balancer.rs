use crate::AirspyhfComplexFloat;

/// Minimal placeholder for the complex IQ balancer used in the C driver.
/// The full DSP logic is not yet ported.
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

    pub fn process(&mut self, iq: *mut AirspyhfComplexFloat, length: i32) {
        if iq.is_null() || length <= 0 {
            return;
        }
        unsafe {
            let slice = std::slice::from_raw_parts_mut(iq, length as usize);
            let scale = 1.0 / ((length - 1) as f32);
            for (i, s) in slice.iter_mut().enumerate() {
                let p = (i as f32 * self.last_phase + (length - 1 - i as i32) as f32 * self.phase)
                    * scale;
                let a = (i as f32 * self.last_amplitude
                    + (length - 1 - i as i32) as f32 * self.amplitude)
                    * scale;
                let re = s.re;
                let im = s.im;
                s.re += p * im;
                s.im += p * re;
                s.re *= 1.0 + a;
                s.im *= 1.0 - a;
            }
        }
        self.last_phase = self.phase;
        self.last_amplitude = self.amplitude;
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
