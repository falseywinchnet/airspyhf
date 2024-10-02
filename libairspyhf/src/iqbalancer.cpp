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




static void adjust_benchmark_no_sum(struct iq_balancer_t* iq_balancer, complex_t* RESTRICT iq, float phase, float amplitude)
{
	int i;

	
		VECTORIZE_LOOP
		for (i = 0; i < FFTBins; i++)
		{
			float re = iq[i].re;
			float im = iq[i].im;

			iq[i].re += phase * im;
			iq[i].im += phase * re;

			iq[i].re *= 1 + amplitude;
			iq[i].im *= 1 - amplitude;
		}
}

static float adjust_benchmark_return_sum(struct iq_balancer_t* iq_balancer, complex_t* RESTRICT iq, float phase, float amplitude)
{
	int i;
	double sum = 0;

	
		VECTORIZE_LOOP
		for (i = 0; i < FFTBins; i++)
		{
			float re = iq[i].re;
			float im = iq[i].im;

			iq[i].re += phase * im;
			iq[i].im += phase * re;

			iq[i].re *= 1 + amplitude;
			iq[i].im *= 1 - amplitude;
			sum += re * re + im * im;
		}
	return (float)sum;
}


static complex_t multiply_complex_complex(complex_t* RESTRICT a, const complex_t* RESTRICT b)
{
	complex_t result;
	result.re = a->re * b->re - a->im * b->im;
	result.im = a->im * b->re + a->re * b->im;
	return result;
}


static int compute_corr(struct iq_balancer_t* iq_balancer, complex_t* RESTRICT iq, complex_t* RESTRICT  ccorr, int length, int step)
{
	complex_t cc;
	complex_t* RESTRICT fftPtr = __fft_mem;
	float* RESTRICT boost = iq_balancer->boost;
	float* RESTRICT boost_window = __boost_window;
	int* RESTRICT power_flag = iq_balancer->power_flag;

	int n, m = 0;
	int i, j;
	int count = 0;
	float power = 0.0f;
	float phase = iq_balancer->phase + step * PhaseStep;
	float amplitude = iq_balancer->amplitude + step * AmplitudeStep;
	int optimal_bin = iq_balancer->optimal_bin;
	if (step == 0)
	{
		for (n = 0, m = 0; n <= length - FFTBins && m < iq_balancer->fft_integration; n += FFTBins / iq_balancer->fft_overlap, m++)
		{
			memcpy(fftPtr, iq + n, FFTBins * sizeof(complex_t));

				power = adjust_benchmark_return_sum(iq_balancer, fftPtr, phase, amplitude);

				if (power > MinimumPower)
				{
					power_flag[m] = 1;
					iq_balancer->integrated_total_power += power;
				}
				else
				{
					power_flag[m] = 0;
				}
			if (power_flag[m] == 1)
			{
				count++;
				window(fftPtr, FFTBins);
				fft(fftPtr, FFTBins);
				for (i = EdgeBinsToSkip, j = FFTBins - EdgeBinsToSkip; i <= FFTBins - EdgeBinsToSkip; i++, j--)
				{
					cc = multiply_complex_complex(fftPtr + i, fftPtr + j);
					ccorr[i].re += cc.re;
					ccorr[i].im += cc.im;

					ccorr[j].re = ccorr[i].re;
					ccorr[j].im = ccorr[i].im;
				}


				if (optimal_bin == FFTBins / 2) {
					
					for (i = EdgeBinsToSkip; i <= FFTBins - EdgeBinsToSkip; i++)
					{
						power = fftPtr[i].re * fftPtr[i].re + fftPtr[i].im * fftPtr[i].im;
						boost[i] += power;
						iq_balancer->integrated_image_power += power;
					}
				}
				else {
						for (i = EdgeBinsToSkip; i <= FFTBins - EdgeBinsToSkip; i++)
						{
							power = fftPtr[i].re * fftPtr[i].re + fftPtr[i].im * fftPtr[i].im;
							boost[i] += power;
							iq_balancer->integrated_image_power += power * boost_window[abs(FFTBins - i - optimal_bin)];
						}
				}
				
			}
		}
	}
	else
	{
		for (n = 0, m = 0; n <= length - FFTBins && m < iq_balancer->fft_integration; n += FFTBins / iq_balancer->fft_overlap, m++)
		{
			memcpy(fftPtr, iq + n, FFTBins * sizeof(complex_t));
			adjust_benchmark_no_sum(iq_balancer, fftPtr, phase, amplitude);
			if (iq_balancer->power_flag[m] == 1)
			{
				count++;
				window(fftPtr, FFTBins);
				fft(fftPtr, FFTBins);
				
				for (i = EdgeBinsToSkip, j = FFTBins - EdgeBinsToSkip; i <= FFTBins - EdgeBinsToSkip; i++, j--)
				{
					cc = multiply_complex_complex(fftPtr + i, fftPtr + j);
					ccorr[i].re += cc.re;
					ccorr[i].im += cc.im;

					ccorr[j].re = ccorr[i].re;
					ccorr[j].im = ccorr[i].im;
				}
			}
		}
	}
	return count;
}

static complex_t utility(struct iq_balancer_t* iq_balancer, complex_t* RESTRICT  ccorr)
{
	int i;
	int j;
	float* RESTRICT boost_window = __boost_window;
	float* RESTRICT boost = iq_balancer->boost;

	float invskip = 1.0f / EdgeBinsToSkip;
	complex_t acc = { 0, 0 };
	double im = 0.0;
	double re = 0.0;
	for (i = EdgeBinsToSkip, j = FFTBins - EdgeBinsToSkip; i <= FFTBins - EdgeBinsToSkip; i++, j--)
	{
		int distance = abs(i - FFTBins / 2);
		if (distance > CenterBinsToSkip)
		{
			float weight = (distance > EdgeBinsToSkip) ? 1.0f : (distance * invskip);
			if (iq_balancer->optimal_bin != FFTBins / 2)
			{
				weight *= boost_window[abs(iq_balancer->optimal_bin - i)];
			}
			weight *= boost[j] / (boost[i] + EPSILON);
			re += ccorr[i].re * weight;
			im += ccorr[i].im * weight;
		}
	}
	acc.re = re;
	acc.im = im;
	return acc;
}


static float compute_cost_function(struct iq_balancer_t* iq_balancer, complex_t* iq, int length, float phase, float amplitude) {
	int i;
	double image_power = 0.0f;
	complex_t* RESTRICT fftPtr = __fft_mem;

	// Allocate memory for local FFT buffer if needed


	// Copy the first FFTBins samples to the FFT buffer
	memcpy(fftPtr, iq + length, FFTBins * sizeof(complex_t));

	// Apply phase and amplitude corrections
	for (i = 0; i < FFTBins; i++) {
		float re = fftPtr[i].re;
		float im = fftPtr[i].im;

		// Apply phase correction
		fftPtr[i].re += phase * im;
		fftPtr[i].im += phase * re;

		// Apply amplitude correction
		fftPtr[i].re *= 1 + amplitude;
		fftPtr[i].im *= 1 - amplitude;
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

static float find_elliptical_compromise(float p1, float a1, float p2, float a2, float target_p, float target_a)
{
	// This is a simplified approach. A more sophisticated method might be needed for optimal results.
	float t = 0.0f;
	float best_t = 0.0f;
	float min_distance = INFINITY;

	for (int i = 0; i <= 100; i++) {
		t = i / 100.0f;
		float p = p1 + t * (p2 - p1);
		float a = a1 + t * (a2 - a1);
		float distance = sqrtf((p - target_p) * (p - target_p) + (a - target_a) * (a - target_a));
		if (distance < min_distance) {
			min_distance = distance;
			best_t = t;
		}
	}

	return best_t;
}

static void apply_correction(complex_t* sample, float phase, float amplitude)
{
	float re = sample->re;
	float im = sample->im;

	// Apply phase correction
	sample->re += phase * im;
	sample->im += phase * re;

	// Apply amplitude correction
	sample->re *= 1 + amplitude;
	sample->im *= 1 - amplitude;
}
static void estimate_imbalance(struct iq_balancer_t* iq_balancer, complex_t* iq, int length)
{
	int i, j;
	float amplitude, phase, mu;
	complex_t a, b;

	if (iq_balancer->reset_flag)
	{
		iq_balancer->reset_flag = 0;
		iq_balancer->no_of_avg = -BuffersToSkipOnReset;
		iq_balancer->maximum_image_power = 0;
	}

	if (iq_balancer->no_of_avg < 0)
	{
		iq_balancer->no_of_avg++;
		return;
	}
	else if (iq_balancer->no_of_avg == 0)
	{
		iq_balancer->integrated_image_power = 0;
		iq_balancer->integrated_total_power = 0;
		memset(iq_balancer->boost, 0, FFTBins * sizeof(float));
		memset(iq_balancer->corr, 0, FFTBins * sizeof(complex_t));
		memset(iq_balancer->corr_plus, 0, FFTBins * sizeof(complex_t));
	}

	iq_balancer->maximum_image_power *= MaxPowerDecay;

	i = compute_corr(iq_balancer, iq, iq_balancer->corr, length, 0);
	if (i == 0)
		return;

	iq_balancer->no_of_avg += i;
	compute_corr(iq_balancer, iq, iq_balancer->corr_plus, length, 1);

	if (iq_balancer->no_of_avg <= iq_balancer->correlation_integration * iq_balancer->fft_integration)
		return;

	iq_balancer->no_of_avg = 0;

	if (iq_balancer->optimal_bin == FFTBins / 2)
	{
		if (iq_balancer->integrated_total_power < iq_balancer->maximum_image_power)
			return;
		iq_balancer->maximum_image_power = iq_balancer->integrated_total_power;
	}
	else
	{
		if (iq_balancer->integrated_image_power - iq_balancer->integrated_total_power * BoostWindowNorm < iq_balancer->maximum_image_power * PowerThreshold)
			return;
		iq_balancer->maximum_image_power = iq_balancer->integrated_image_power - iq_balancer->integrated_total_power * BoostWindowNorm;
	}

	a = utility(iq_balancer, iq_balancer->corr);
	b = utility(iq_balancer, iq_balancer->corr_plus);

	mu = a.im - b.im;
	if (fabs(mu) > MinDeltaMu)
	{
		mu = a.im / mu;
		if (mu < -MaxMu)
			mu = -MaxMu;
		else if (mu > MaxMu)
			mu = MaxMu;
	}
	else
	{
		mu = 0;
	}

	phase = iq_balancer->phase + PhaseStep * mu;

	mu = a.re - b.re;
	if (fabs(mu) > MinDeltaMu)
	{
		mu = a.re / mu;
		if (mu < -MaxMu)
			mu = -MaxMu;
		else if (mu > MaxMu)
			mu = MaxMu;
	}
	else
	{
		mu = 0;
	}

	amplitude = iq_balancer->amplitude + AmplitudeStep * mu;

	if (iq_balancer->no_of_raw < MaxLookback)
		iq_balancer->no_of_raw++;
	iq_balancer->raw_amplitudes[iq_balancer->raw_ptr] = amplitude;
	iq_balancer->raw_phases[iq_balancer->raw_ptr] = phase;
	i = iq_balancer->raw_ptr;
	for (j = 0; j < iq_balancer->no_of_raw - 1; j++)
	{
		i = (i + MaxLookback - 1) & (MaxLookback - 1);
		phase += iq_balancer->raw_phases[i];
		amplitude += iq_balancer->raw_amplitudes[i];
	}
	phase /= iq_balancer->no_of_raw;
	amplitude /= iq_balancer->no_of_raw;
	iq_balancer->raw_ptr = (iq_balancer->raw_ptr + 1) & (MaxLookback - 1);


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

			prev_J = J;

			// Early stopping if improvement is negligible
			if (i > 0 && fabs(J - prev_J) < CONVERGENCE_THRESHOLD * fabs(prev_J)) {
				break;
			}
		}

	// Apply the new phase and amplitude to a copy of the first element
	complex_t test_sample = iq[0];
	apply_correction(&test_sample, phase, amplitude);

	float test_phase = atan2f(test_sample.im, test_sample.re);
	float test_amplitude = sqrtf(test_sample.re * test_sample.re + test_sample.im * test_sample.im);

	// Check if the new correction brings us closer to the last frame's end
	float current_phase_diff = fabsf(iq_balancer->last_frame_end_phase - atan2f(iq[0].im, iq[0].re));
	float current_amplitude_diff = fabsf(iq_balancer->last_frame_end_amplitude - sqrtf(iq[0].re * iq[0].re + iq[0].im * iq[0].im));
	float new_phase_diff = fabsf(iq_balancer->last_frame_end_phase - test_phase);
	float new_amplitude_diff = fabsf(iq_balancer->last_frame_end_amplitude - test_amplitude);

	if (new_phase_diff <= current_phase_diff && new_amplitude_diff <= current_amplitude_diff) {
		// New correction improves continuity, use it directly
		iq_balancer->phase = phase;
		iq_balancer->amplitude = amplitude;
	}
	else {
		// Find a compromise using elliptical calculation
		float t = find_elliptical_compromise(iq_balancer->phase, iq_balancer->amplitude,
			phase, amplitude,
			iq_balancer->last_frame_end_phase, iq_balancer->last_frame_end_amplitude);
		iq_balancer->phase = iq_balancer->phase + t * (phase - iq_balancer->phase);
		iq_balancer->amplitude = iq_balancer->amplitude + t * (amplitude - iq_balancer->amplitude);
	}
}






static void adjust_phase_amplitude(struct iq_balancer_t* iq_balancer, complex_t* RESTRICT iq, int length)
{
	int i;
	float scale = 1.0f / (length - 1);

		VECTORIZE_LOOP
		for (i = 0; i < length; i++)
		{
			float phase = (i * iq_balancer->last_phase + (length - 1 - i) * iq_balancer->phase) * scale;
			float amplitude = (i * iq_balancer->last_amplitude + (length - 1 - i) * iq_balancer->amplitude) * scale;

			float re = iq[i].re;
			float im = iq[i].im;

			iq[i].re += phase * im;
			iq[i].im += phase * re;

			iq[i].re *= 1 + amplitude;
			iq[i].im *= 1 - amplitude;
		}

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
	iq_balancer->last_frame_end_phase = atan2f(iq[length - 1].im, iq[length - 1].re);
	iq_balancer->last_frame_end_amplitude = sqrtf(iq[length - 1].re * iq[length - 1].re + iq[length - 1].im * iq[length - 1].im);

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
