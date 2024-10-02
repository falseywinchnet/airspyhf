/*
Copyright (c) 2016-2023, Youssef Touil <youssef@airspy.com>
Copyright (c) 2018, Leif Asbrink <leif@sm5bsz.com>
Copyright (C) 2024, Joshuah Rainstar <joshuah.rainstar@gmail.com>
Contributions to this work were provided by OpenAI Codex, an artifical general intelligence.



All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

		Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
		Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the
		documentation and/or other materials provided with the distribution.
		Neither the name of Airspy HF+ nor the names of its contributors may be used to endorse or promote products derived from this software
		without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "iqbalancer.h"


#if defined(_MSC_VER) && !defined(__clang__)
#define RESTRICT __restrict
#elif defined(__GNUC__) 
#define RESTRICT restrict
#elif defined(__clang__)
#define RESTRICT __restrict__
#else
#define RESTRICT 
#endif

#if defined(_MSC_VER) && !defined(__clang__)
#define VECTORIZE_LOOP _Pragma("loop(ivdep, unroll)")
#elif defined(__clang__)
#define VECTORIZE_LOOP _Pragma("clang loop vectorize(enable) unroll(enable)")
#elif defined(__GNUC__)
#define VECTORIZE_LOOP _Pragma("GCC ivdep") _Pragma("GCC unroll 4")
#else
#define VECTORIZE_LOOP
#endif

#ifndef MATH_PI
#define MATH_PI 3.14159265359
#endif

#define EPSILON 1e-4f
#define WorkingBufferLength (FFTBins * (1 + FFTIntegration / FFTOverlap))

#define BUFFER_SIZE 4096
#define TRIPLE_BUFFER_SIZE (BUFFER_SIZE * 3)
#define MAX_EXTREMA BUFFER_SIZE+2  // Adjust as needed
#define HISTORY_SIZE 5

struct iq_balancer_t
{
	float last_frame_end_phase;
	float last_frame_end_amplitude;
	double phase;
	double last_phase;
	double phase_new;
	double amplitude_new;
	double learning_rate;  // Controls how fast we update based on the gradient
	double amplitude;
	double last_amplitude;
	double iavg;
	double qavg;
	double integrated_total_power;
	double integrated_image_power;
	float maximum_image_power;
	float raw_phases[MaxLookback];
	float raw_amplitudes[MaxLookback];
	int skipped_buffers;
	int buffers_to_skip;
	int working_buffer_pos;
	int fft_integration;
	int fft_overlap;
	int correlation_integration;
	float phase_history[HISTORY_SIZE];
	float amplitude_history[HISTORY_SIZE];
	int history_index = 0;
	int no_of_avg;
	int no_of_raw;
	int raw_ptr;
	int optimal_bin;
	int reset_flag;
	int power_flag[FFTIntegration];
	float alpha = 1e-4;
	complex_t* corr;
	complex_t* corr_plus;
	complex_t* working_buffer;
	float* boost;

};


static uint8_t __lib_initialized = 0;
static complex_t __fft_mem[FFTBins];



static float __fft_window[FFTBins];
static float __boost_window[FFTBins];
static complex_t twiddle_factors[FFTBins/2];
#define HISTORY_SIZE 5  // Number of historical points to keep



static void __init_library()
{
	int i;

	if (__lib_initialized)
	{
		return;
	}

	const int length = FFTBins - 1;

	/*for (i = 0; i <= length; i++)
	{
		__fft_window[i] = (float)(
			+0.35875f
			- 0.48829f * cos(2.0 * MATH_PI * i / length)
			+ 0.14128f * cos(4.0 * MATH_PI * i / length)
			- 0.01168f * cos(6.0 * MATH_PI * i / length)
			);
		__boost_window[i] = (float)(1.0 / BoostFactor + 1.0 / exp(pow(i * 2.0 / BinsToOptimize, 2.0)));
	}*/


	//Albrecht 10 terms 
	VECTORIZE_LOOP
	for (i = 0; i <= length; i++)
	{
		__fft_window[i] = (float)(
			+0.2260721603916653632706
			- 0.3865459981017629121952 * cos(2.0 * MATH_PI * i / length)
			+ 0.2402581984804387251180 * cos(4.0 * MATH_PI * i / length)
			- 0.1067615081338829512123 * cos(6.0 * MATH_PI * i / length)
			+ 0.03286350853942572526446 * cos(8.0 * MATH_PI * i / length)
			- 0.006643005438025320617263 * cos(10.0 * MATH_PI * i / length)
			+ 0.0008050608438216912274761 * cos(12.0 * MATH_PI * i / length)
			- 0.00004948590944767209041847 * cos(14.0 * MATH_PI * i / length)
			+ 0.000001071744648495088830343 * cos(16.0 * MATH_PI * i / length)
			- 0.000000002416881143872775668631 * cos(18.0 * MATH_PI * i / length)
			);
		__boost_window[i] = (float)(1.0 / BoostFactor + 1.0 / exp(pow(i * 2.0 / BinsToOptimize, 2.0)));
	}
	VECTORIZE_LOOP
	for (int i = 0; i < FFTBins / 2; i++) {
		float angle = -2.0f * MATH_PI * i / FFTBins;
		twiddle_factors[i].re = cosf(angle);
		twiddle_factors[i].im = sinf(angle);
	}

	__lib_initialized = 1;
}

static void window(complex_t* RESTRICT buffer, int length)
{
	int i;

		VECTORIZE_LOOP
		for (i = 0; i < length; i++)
		{
			buffer[i].re *= __fft_window[i];
			buffer[i].im *= __fft_window[i];
		}
}

static void fft(complex_t* buffer, int length)
{
	int nm1 = length - 1;
	int nd2 = length / 2;
	int m = 0;
	complex_t t;

	// Calculate m = log2(length)
	for (int i = length; i > 1; i >>= 1) {
		++m;
	}

	// Bit-reverse ordering
	int j = nd2;
	for (int i = 1; i < nm1; ++i) {
		if (i < j) {
			t = buffer[j];
			buffer[j] = buffer[i];
			buffer[i] = t;
		}
		int k = nd2;
		while (k <= j) {
			j -= k;
			k >>= 1;
		}
		j += k;
	}

	// FFT computation
	for (int l = 1; l <= m; ++l) {
		int le = 1 << l;
		int le2 = le >> 1;
		int step = length / le;

		for (int j = 0; j < le2; ++j) {
			for (int i = j; i < length; i += le) {
				int ip = i + le2;
				complex_t twiddle = twiddle_factors[j * step];

				t.re = twiddle.re * buffer[ip].re - twiddle.im * buffer[ip].im;
				t.im = twiddle.im * buffer[ip].re + twiddle.re * buffer[ip].im;

				buffer[ip].re = buffer[i].re - t.re;
				buffer[ip].im = buffer[i].im - t.im;

				buffer[i].re += t.re;
				buffer[i].im += t.im;
			}
		}
	}

	// Swap real and imaginary parts
	for (int i = 0; i < nd2; i++) {
		j = nd2 + i;
		t = buffer[i];
		buffer[i] = buffer[j];
		buffer[j] = t;
	}
}
void compute_uniform_spline_coefficients(const float* y, float h, float* a, float* b, float* c, float* d) {
	int i;
	float inv_h = 1.0f / h;
	float inv_h2 = inv_h * inv_h;
	constexpr int n = HISTORY_SIZE;

	constexpr int m = n - 2; // Number of equations

	float rhs[m];
	for (i = 1; i <= m; i++) {
		rhs[i - 1] = 6.0f * (y[i + 1] - 2.0f * y[i] + y[i - 1]) * inv_h2;
	}

	// Forward elimination
	float c_prime[m];
	float rhs_prime[m];
	float m_diag = 4.0f;

	c_prime[0] = 1.0f / m_diag;
	rhs_prime[0] = rhs[0] / m_diag;

	for (i = 1; i < m; i++) {
		m_diag = 4.0f - c_prime[i - 1];
		c_prime[i] = 1.0f / m_diag;
		rhs_prime[i] = (rhs[i] - rhs_prime[i - 1]) / m_diag;
	}

	// Back substitution
	float M[n]; // Second derivatives
	M[0] = M[n - 1] = 0.0f; // Natural spline boundary conditions

	M[n - 2] = rhs_prime[m - 1];

	for (i = m - 2; i >= 0; i--) {
		M[i + 1] = rhs_prime[i] - c_prime[i] * M[i + 2];
	}

	// Compute spline coefficients
	for (i = 0; i < n - 1; i++) {
		a[i] = y[i];
		b[i] = (y[i + 1] - y[i]) * inv_h - h * (2.0f * M[i] + M[i + 1]) / 6.0f;
		c[i] = M[i] / 2.0f;
		d[i] = (M[i + 1] - M[i]) / (6.0f * h);
	}
}
float evaluate_uniform_spline(float x, float h, const float* a, const float* b, const float* c, const float* d, int n) {
	int seg = (int)(x / h);
	if (seg >= n - 1) seg = n - 2; // Clamp to last segment

	float dx = x - seg * h;

	return a[seg] + b[seg] * dx + c[seg] * dx * dx + d[seg] * dx * dx * dx;
}
void interpolate_phase_amplitude(struct iq_balancer_t* iq_balancer, complex_t* RESTRICT iq, int length) {
	// Assume HISTORY_SIZE >= 4
	constexpr int n = HISTORY_SIZE;
	float h = 1.0f; // Since time indices are 0, 1, 2, ..., n - 1

	float phase_values[n];
	float amplitude_values[n];

	// Extract phase and amplitude history
	for (int i = 0; i < n; i++) {
		int index = (iq_balancer->history_index + i) % n;
		phase_values[i] = iq_balancer->phase_history[index];
		amplitude_values[i] = iq_balancer->amplitude_history[index];
	}

	// Compute spline coefficients
	float a_phase[n - 1], b_phase[n - 1], c_phase[n - 1], d_phase[n - 1];
	float a_amp[n - 1], b_amp[n - 1], c_amp[n - 1], d_amp[n - 1];

	compute_uniform_spline_coefficients(phase_values, h, a_phase, b_phase, c_phase, d_phase);
	compute_uniform_spline_coefficients(amplitude_values, h, a_amp, b_amp, c_amp, d_amp);

	// Apply corrections
	float total_time = (n - 1) * h;
	float time_per_sample = total_time / (length - 1);

	for (int i = 0; i < length; i++) {
		float x = i * time_per_sample;

		float phase = evaluate_uniform_spline(x, h, a_phase, b_phase, c_phase, d_phase, n);
		float amplitude = evaluate_uniform_spline(x, h, a_amp, b_amp, c_amp, d_amp, n);

		// Apply corrections
		float re = iq[i].re;
		float im = iq[i].im;

		iq[i].re += phase * im;
		iq[i].im += phase * re;

		iq[i].re *= 1.0f + amplitude;
		iq[i].im *= 1.0f - amplitude;
	}
}



static float compute_cost_function(struct iq_balancer_t* iq_balancer, complex_t* iq, int length, float phase1, float amplitude1) {
	int i;
	double image_power = 0.0f;
	complex_t* RESTRICT fftPtr = __fft_mem;

	// Copy the first FFTBins samples to the FFT buffer
	memcpy(fftPtr, iq, FFTBins * sizeof(complex_t));

	// Prepare data for interpolation
	// We need at least 4 points for cubic spline interpolation
	int n = HISTORY_SIZE;
	if (n < 4) {
		// Fall back to linear interpolation if not enough data points
		n = 2;
	}
	float h = 1.0f; // Uniform spacing between points

	float phase_values[n];
	float amplitude_values[n];

	// Build the phase and amplitude history
	// The last point is the current estimate (phase1, amplitude1)
	for (int j = 0; j < n - 1; j++) {
		int index = (iq_balancer->history_index + j) % HISTORY_SIZE;
		phase_values[j] = iq_balancer->phase_history[index];
		amplitude_values[j] = iq_balancer->amplitude_history[index];
	}
	// Add the current estimates as the last point
	phase_values[n - 1] = phase1;
	amplitude_values[n - 1] = amplitude1;

	// Compute spline coefficients
	float a_phase[n - 1], b_phase[n - 1], c_phase[n - 1], d_phase[n - 1];
	float a_amp[n - 1], b_amp[n - 1], c_amp[n - 1], d_amp[n - 1];

	compute_uniform_spline_coefficients(phase_values, h, a_phase, b_phase, c_phase, d_phase);
	compute_uniform_spline_coefficients(amplitude_values, h, a_amp, b_amp, c_amp, d_amp);

	// Apply spline-based corrections
	float total_time = (n - 1) * h;
	float time_per_sample = total_time / (FFTBins - 1);

	for (i = 0; i < FFTBins; i++) {
		float x = i * time_per_sample;

		// Evaluate spline to get interpolated phase and amplitude
		float phase = evaluate_uniform_spline(x, h, a_phase, b_phase, c_phase, d_phase, n);
		float amplitude = evaluate_uniform_spline(x, h, a_amp, b_amp, c_amp, d_amp, n);

		// Apply corrections
		float re = fftPtr[i].re;
		float im = fftPtr[i].im;

		// Phase correction
		fftPtr[i].re += phase * im;
		fftPtr[i].im += phase * re;

		// Amplitude correction
		fftPtr[i].re *= 1.0f + amplitude;
		fftPtr[i].im *= 1.0f - amplitude;
	}

	// Apply window function
	window(fftPtr, FFTBins);

	// Perform FFT
	fft(fftPtr, FFTBins);

	// Compute image power
	int k_start = FFTBins / 2 + CenterBinsToSkip; // Start of image frequencies
	int k_end = FFTBins - EdgeBinsToSkip;         // End of image frequencies

	for (i = k_start; i < k_end; i++) {
		float re = fftPtr[i].re;
		float im = fftPtr[i].im;
		image_power += re * re + im * im;
	}
	return image_power;
}

static void estimate_imbalance(struct iq_balancer_t* iq_balancer, complex_t* iq, int length)
{
	float amplitude, phase;
	amplitude = iq_balancer->amplitude;
	phase = iq_balancer->phase;



	const int MAX_ITERATIONS = 10;  // Adjust as needed
	const float CONVERGENCE_THRESHOLD = 1e-6;  // Adjust as needed
	const float LEARNING_RATE_DECAY = 0.9;    // More aggressive decay
	float prev_J = INFINITY;

	for (int i = 0; i < MAX_ITERATIONS; i++) {
		// Compute cost with current phase and amplitude
		float J = compute_cost_function(iq_balancer, iq, length, phase, amplitude);

		// Compute costs with perturbed phase and amplitude
		float J_phi = compute_cost_function(iq_balancer, iq, length, phase + PhaseStep, amplitude);
		float J_A = compute_cost_function(iq_balancer, iq, length, phase, amplitude + AmplitudeStep);

		// Calculate gradients
		float gradient_phi = (J_phi - J) / PhaseStep;
		float gradient_A = (J_A - J) / AmplitudeStep;

		// Estimate effectiveness of current learning rate
		float gradient_magnitude = sqrt(gradient_phi * gradient_phi + gradient_A * gradient_A);
		float proposed_step = iq_balancer->learning_rate * gradient_magnitude;

		// Adjust learning rate based on gradient magnitude and cost change
		if (J < prev_J) {
			if (proposed_step < 0.1 * (PhaseStep + AmplitudeStep) / 2) {
				iq_balancer->learning_rate /= LEARNING_RATE_DECAY;  // Increase if step is too small
			}
			else if (proposed_step > 10 * (PhaseStep + AmplitudeStep) / 2) {
				iq_balancer->learning_rate *= LEARNING_RATE_DECAY;  // Decrease if step is too large
			}
		}
		else {
			iq_balancer->learning_rate *= LEARNING_RATE_DECAY * LEARNING_RATE_DECAY;  // Decrease more if cost increased
		}

		// Update phase and amplitude
		phase -= iq_balancer->learning_rate * gradient_phi;
		amplitude -= iq_balancer->learning_rate * gradient_A;
		phase = fmod(iq_balancer->phase, 2 * MATH_PI);

		prev_J = J;

		// Early stopping if improvement is negligible
		if (i > 0 && fabs(J - prev_J) < CONVERGENCE_THRESHOLD * fabs(prev_J)) {
			break;
		}
	}

	iq_balancer->phase = phase;
	iq_balancer->amplitude = amplitude;

}



static void adjust_phase_amplitude(struct iq_balancer_t* iq_balancer, complex_t* RESTRICT iq, int length)
{
	int i;
	float frame_time = (float)(length - 1);

	// Values at control points
	float phase0 = iq_balancer->last_phase;
	float phase1 = iq_balancer->phase;
	float amplitude0 = iq_balancer->last_amplitude;
	float amplitude1 = iq_balancer->amplitude;

	// Derivatives at control points (assuming constant rate)
	float phase_derivative = (phase1 - phase0) / frame_time;
	float amplitude_derivative = (amplitude1 - amplitude0) / frame_time;

	VECTORIZE_LOOP
		for (i = 0; i < length; i++)
		{
			float t = (float)i / frame_time;

			// Hermite basis functions
			float h00 = 2 * t * t * t - 3 * t * t + 1;
			float h10 = t * t * t - 2 * t * t + t;
			float h01 = -2 * t * t * t + 3 * t * t;
			float h11 = t * t * t - t * t;

			// Interpolated phase and amplitude
			float phase = h00 * phase0 + h10 * phase_derivative + h01 * phase1 + h11 * phase_derivative;
			float amplitude = h00 * amplitude0 + h10 * amplitude_derivative + h01 * amplitude1 + h11 * amplitude_derivative;

			// Apply corrections
			float re = iq[i].re;
			float im = iq[i].im;

			// Phase correction
			iq[i].re += phase * im;
			iq[i].im += phase * re;

			// Amplitude correction
			iq[i].re *= 1 + amplitude;
			iq[i].im *= 1 - amplitude;
		}

	// Update last values
	iq_balancer->last_phase = iq_balancer->phase;
	iq_balancer->last_amplitude = iq_balancer->amplitude;
}



static void cancel_dc(struct iq_balancer_t* iq_balancer, complex_t* RESTRICT iq, int length, float alpha)
{
	int i;
	double iavg = iq_balancer->iavg;
	double qavg = iq_balancer->qavg;//must use double accumulator because of rounding error over time

	for (i = 0; i < length; i++)
	{
		iavg = (1 - alpha) * iavg + alpha * iq[i].re;
		qavg = (1 - alpha) * qavg + alpha * iq[i].im;
		iq[i].re -= iavg;
		iq[i].im -= qavg;
	}
	iq_balancer->iavg = iavg;
	iq_balancer->qavg = qavg;
}


void ADDCALL iq_balancer_process(struct iq_balancer_t* iq_balancer, complex_t*  iq, int length)
{
	int count;
	cancel_dc(iq_balancer, iq, length,1e-4f);

	count = WorkingBufferLength - iq_balancer->working_buffer_pos;
		if (count >= length)
		{
			count = length;
		}
		memcpy(iq_balancer->working_buffer + iq_balancer->working_buffer_pos, iq, count * sizeof(complex_t));
		iq_balancer->working_buffer_pos += count;
		if (iq_balancer->working_buffer_pos >= WorkingBufferLength)
		{
			iq_balancer->working_buffer_pos = 0;

			if (++iq_balancer->skipped_buffers > iq_balancer->buffers_to_skip)
			{
				iq_balancer->skipped_buffers = 0;
				estimate_imbalance(iq_balancer, iq_balancer->working_buffer, WorkingBufferLength);

			}
		}

	adjust_phase_amplitude(iq_balancer, iq, length);
	iq_balancer->phase_history[iq_balancer->history_index] = iq_balancer->phase;
	iq_balancer->amplitude_history[iq_balancer->history_index] = iq_balancer->amplitude;

	// Increment and wrap the history index
	iq_balancer->history_index = (iq_balancer->history_index + 1) % HISTORY_SIZE;
}

void ADDCALL iq_balancer_set_optimal_point(struct iq_balancer_t* iq_balancer, float w)
{
	if (w < -0.5f)
	{
		w = -0.5f;
	}
	else if (w > 0.5f)
	{
		w = 0.5f;
	}

	iq_balancer->optimal_bin = (int)floor(FFTBins * (0.5 + w));
	iq_balancer->reset_flag = 1;
}

void ADDCALL iq_balancer_configure(struct iq_balancer_t* iq_balancer, int buffers_to_skip, int fft_integration, int fft_overlap, int correlation_integration)
{
	iq_balancer->buffers_to_skip = buffers_to_skip;
	iq_balancer->fft_integration = fft_integration;
	iq_balancer->fft_overlap = fft_overlap;
	iq_balancer->correlation_integration = correlation_integration;

	memset(iq_balancer->power_flag, 0, iq_balancer->fft_integration * sizeof(int));

	iq_balancer->reset_flag = 1;
}

struct iq_balancer_t* ADDCALL iq_balancer_create(float initial_phase, float initial_amplitude)
{
	struct iq_balancer_t* instance = (struct iq_balancer_t*)malloc(sizeof(struct iq_balancer_t));
	if (instance == NULL)
	{
		// Handle memory allocation failure
		return NULL;
	}
	
	memset(instance, 0, sizeof(struct iq_balancer_t));
	instance->phase = initial_phase;
	instance->amplitude = initial_amplitude;
	instance->learning_rate = 0.1f;//0.0898f;  // delta t / bandwidth
	instance->optimal_bin = FFTBins / 2;

	instance->buffers_to_skip = BuffersToSkip;
	instance->fft_integration = FFTIntegration;
	instance->fft_overlap = FFTOverlap;
	instance->correlation_integration = CorrelationIntegration;

	instance->corr = (complex_t*)malloc(FFTBins * sizeof(complex_t));
	instance->corr_plus = (complex_t*)malloc(FFTBins * sizeof(complex_t));
	instance->working_buffer = (complex_t*)malloc(WorkingBufferLength * sizeof(complex_t));

	instance->boost = (float*)malloc(FFTBins * sizeof(float));

	__init_library();
	return instance;
}

void ADDCALL iq_balancer_destroy(struct iq_balancer_t* iq_balancer)
{
	free(iq_balancer->corr);
	free(iq_balancer->corr_plus);
	free(iq_balancer->working_buffer);
	free(iq_balancer->boost);
	free(iq_balancer);
}
