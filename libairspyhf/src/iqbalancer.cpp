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
#define VECTORIZE_LOOP _Pragma("loop(ivdep)")
#elif defined(__clang__)
#define VECTORIZE_LOOP _Pragma("clang loop vectorize(enable)")
#elif defined(__GNUC__)
#define VECTORIZE_LOOP _Pragma("GCC ivdep"))
#else
#define VECTORIZE_LOOP
#endif

#ifndef MATH_PI
#define MATH_PI 3.14159265359
#endif

#define WorkingBufferLength (FFTBins * (1 + FFTIntegration / FFTOverlap))



struct iq_balancer_t
{
	double phase;
	double last_phase;
	double phase_new;
	double amplitude;
	double last_amplitude;
	double iavg;
	double qavg;
	double integrated_total_power;
	double integrated_image_power;
	double maximum_image_power;
	float raw_phases[MaxLookback];
	float raw_amplitudes[MaxLookback];
	int skipped_buffers;
	int buffers_to_skip;
	int working_buffer_pos;
	int fft_integration;
	int fft_overlap;
	int correlation_integration;
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

static complex_t __fft_mem[FFTBins];
static uint8_t __lib_initialized = 0;



float dfloat(double value) {
	return static_cast<float>(value);
}

static float __fft_window[FFTBins];
static float __boost_window[FFTBins];
static float __distance_weights[FFTBins];
static complex_t twiddle_factors[FFTBins / 2];
#define HISTORY_SIZE 5  // Number of historical points to keep



static void __init_library()
{
	int i;

	if (__lib_initialized)
	{
		return;
	}

	const int length = FFTBins - 1;

	for (i = 0; i <= length; i++)
	{
		__fft_window[i] = (float)(
			+0.35875f
			- 0.48829f * cos(2.0 * MATH_PI * i / length)
			+ 0.14128f * cos(4.0 * MATH_PI * i / length)
			- 0.01168f * cos(6.0 * MATH_PI * i / length)
			);
		__boost_window[i] = (float)(1.0 / BoostFactor + 1.0 / exp(pow(i * 2.0 / BinsToOptimize, 2.0)));
	}


	//Albrecht 10 terms 

	/*for (i = 0; i <= length; i++)
	{
		__fft_window[i] = (dfloat)(
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
		__boost_window[i] = (dfloat)(1.0 / BoostFactor + 1.0 / exp(pow(i * 2.0 / BinsToOptimize, 2.0)));
	}*/

	for (int i = 0; i < FFTBins / 2; i++) {
		double angle = -2.0 * MATH_PI * i / FFTBins;
		twiddle_factors[i].re = dfloat(cos(angle));
		twiddle_factors[i].im = dfloat(sin(angle));
	}
	int center = FFTBins / 2;
	float invskip = 1.0f / EdgeBinsToSkip;

	VECTORIZE_LOOP
		for (i = EdgeBinsToSkip; i <= center - EdgeBinsToSkip; i++)
		{
			__distance_weights[i] = 1.0f;
		}
	VECTORIZE_LOOP
		for (i = center - EdgeBinsToSkip; i <= FFTBins - EdgeBinsToSkip; i++)
		{
			__distance_weights[i] = 1.0f;
		}

	for (i = center - EdgeBinsToSkip; i <= center - CenterBinsToSkip; i++) {
		int distance = abs(i - center);
		float weight = distance * invskip;
		__distance_weights[i] = weight;
		__distance_weights[FFTBins - i] = weight;
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


static void adjust_phase_amplitude(struct iq_balancer_t* iq_balancer, complex_t* iq, int length)
{
	int i;
	float scale = 1.0f / (length - 1);

	for (i = 0; i < length; i++)
	{
		double phase = (i * iq_balancer->last_phase + (length - 1 - i) * iq_balancer->phase) * scale;
		double amplitude = (i * iq_balancer->last_amplitude + (length - 1 - i) * iq_balancer->amplitude) * scale;

		float re = iq[i].re;
		float im = iq[i].im;

		iq[i].re += dfloat(phase * im);
		iq[i].im += dfloat(phase * re);

		iq[i].re = dfloat(iq[i].re * (1 + amplitude));
		iq[i].im = dfloat(iq[i].im * (1 - amplitude));
	}

	iq_balancer->last_phase = iq_balancer->phase;
	iq_balancer->last_amplitude = iq_balancer->amplitude;
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
	return (dfloat)(sum);
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
	int i;
	int count = 0;
	float power = 0.0f;
	float phase = dfloat(iq_balancer->phase + step * PhaseStep);
	float amplitude = dfloat(iq_balancer->amplitude + step * AmplitudeStep);
	int optimal_bin = iq_balancer->optimal_bin;
	if (step == 0)
	{
		for (n = 0, m = 0; n <= length - FFTBins && m < iq_balancer->fft_integration; n += FFTBins / iq_balancer->fft_overlap, m++)
		{
			power_flag[m] = 0;
			memcpy(fftPtr, iq + n, FFTBins * sizeof(complex_t));

			power = adjust_benchmark_return_sum(iq_balancer, fftPtr, phase, amplitude);

			if (power > MinimumPower)
			{
				power_flag[m] = 1;
				iq_balancer->integrated_total_power += power;
				count++;
				window(fftPtr, FFTBins);
				fft(fftPtr, FFTBins);
				//186, j = 4096-186, i counts to j,
				//j counts to i
				VECTORIZE_LOOP
				for (i = EdgeBinsToSkip; i < FFTBins / 2 - 1; i++) {
						cc = multiply_complex_complex(fftPtr + i, fftPtr + FFTBins - i);
						ccorr[i].re += cc.re;
						ccorr[i].im += cc.im;
					}

				// Mirror the results
				VECTORIZE_LOOP
				for (i = EdgeBinsToSkip; i < FFTBins / 2 - 1; i++) {
					ccorr[FFTBins - i].re = ccorr[i].re;
					ccorr[FFTBins - i].im = ccorr[i].im;
				}
				VECTORIZE_LOOP
					for (i = FFTBins / 2 ; i <= FFTBins - EdgeBinsToSkip; i++) {
						cc = multiply_complex_complex(fftPtr + i, fftPtr + FFTBins - i);
						ccorr[i].re += cc.re;
						ccorr[i].im += cc.im;
					}

				// Mirror the results
				
					for (i = FFTBins / 2; i <= FFTBins - EdgeBinsToSkip; i++) {
						ccorr[FFTBins - i].re = ccorr[i].re;
						ccorr[FFTBins - i].im = ccorr[i].im;
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
			else
			{
				power_flag[m] = 0;
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
				window(fftPtr, FFTBins);
				fft(fftPtr, FFTBins);

				VECTORIZE_LOOP
					for (i = EdgeBinsToSkip; i < FFTBins / 2 - 1; i++) {
						cc = multiply_complex_complex(fftPtr + i, fftPtr + FFTBins - i);
						ccorr[i].re += cc.re;
						ccorr[i].im += cc.im;
					}

				// Mirror the results
				VECTORIZE_LOOP
					for (i = EdgeBinsToSkip; i < FFTBins / 2 - 1; i++) {
						ccorr[FFTBins - i].re = ccorr[i].re;
						ccorr[FFTBins - i].im = ccorr[i].im;
					}
				VECTORIZE_LOOP
					for (i = FFTBins / 2; i <= FFTBins - EdgeBinsToSkip; i++) {
						cc = multiply_complex_complex(fftPtr + i, fftPtr + FFTBins - i);
						ccorr[i].re += cc.re;
						ccorr[i].im += cc.im;
					}

				// Mirror the results
				
					for (i = FFTBins / 2; i <= FFTBins - EdgeBinsToSkip; i++) {
						ccorr[FFTBins - i].re = ccorr[i].re;
						ccorr[FFTBins - i].im = ccorr[i].im;
					}
			}
		}
	}
	return count;
}


static complex_t utility(struct iq_balancer_t* iq_balancer, complex_t* RESTRICT ccorr)
{
	float* RESTRICT boost_window = __boost_window;
	float* RESTRICT distance_weights = __distance_weights;
	float* RESTRICT boost = iq_balancer->boost;
	complex_t acc = { 0, 0 };
	int i;

	// Phase 1: Compute weights for all bins
	int center = FFTBins / 2;
	int optimal_bin = iq_balancer->optimal_bin;

	// Pre-compute distance weights for the entire array


	if (optimal_bin != center) {
		// Compute final weights using vectorizable operations
		VECTORIZE_LOOP
			for (i = EdgeBinsToSkip; i <= center + CenterBinsToSkip; i++) {
				float boost_factor = boost[FFTBins - i] / (boost[i] + EPSILON);

				acc.re += ccorr[i].re * distance_weights[i] * boost_factor * boost_window[abs(optimal_bin - i)];
				acc.im += ccorr[i].im * distance_weights[i] * boost_factor * boost_window[abs(optimal_bin - i)];
			}
		VECTORIZE_LOOP
			for (i = center + CenterBinsToSkip; i <= FFTBins - EdgeBinsToSkip; i++) {
				float boost_factor = boost[FFTBins - i] / (boost[i] + EPSILON);

				acc.re += ccorr[i].re * distance_weights[i] * boost_factor * boost_window[abs(optimal_bin - i)];
				acc.im += ccorr[i].im * distance_weights[i] * boost_factor * boost_window[abs(optimal_bin - i)];

			}
	}
	else {
		VECTORIZE_LOOP
			for (i = EdgeBinsToSkip; i <= center + CenterBinsToSkip; i++) {
				float boost_factor = boost[FFTBins - i] / (boost[i] + EPSILON);

				acc.re += ccorr[i].re * distance_weights[i] * boost_factor;
				acc.im += ccorr[i].im * distance_weights[i] * boost_factor;
			}
		VECTORIZE_LOOP
			for (i = center + CenterBinsToSkip; i <= FFTBins - EdgeBinsToSkip; i++) {
				float boost_factor = boost[FFTBins - i] / (boost[i] + EPSILON);

				acc.re += ccorr[i].re * distance_weights[i] * boost_factor;
				acc.im += ccorr[i].im * distance_weights[i] * boost_factor;
			}

	}

	return acc;
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

	phase = dfloat(iq_balancer->phase + PhaseStep * mu);

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

	amplitude = dfloat(iq_balancer->amplitude + AmplitudeStep * mu);

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

	iq_balancer->phase = phase;
	iq_balancer->amplitude = amplitude;

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
		iq[i].re -= dfloat(iavg);
		iq[i].im -= dfloat(qavg);
	}
	iq_balancer->iavg = iavg;
	iq_balancer->qavg = qavg;
}


void ADDCALL iq_balancer_process(struct iq_balancer_t* iq_balancer, complex_t* iq, int length)
{
	int count;
	cancel_dc(iq_balancer, iq, length, 1e-4f);

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

	iq_balancer->optimal_bin = (int)floor(FFTBins * (0.5f + w));
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
