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
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "iqbalancer.h"

#include <stdint.h>
#if defined(_MSC_VER) // MSVC Compiler

#include <intrin.h>
#pragma intrinsic(_BitScanForward)
#pragma intrinsic(_BitScanReverse)

uint32_t __inline ctz(uint32_t value) // Count trailing zeros
{
	unsigned long trailing_zero = 0;
	if (_BitScanForward(&trailing_zero, value))
	{
		return trailing_zero;
	}
	else
	{
		// If value is zero, return 32 (undefined result)
		return 32;
	}
}

uint32_t __inline clz(uint32_t value) // Count leading zeros
{
	unsigned long leading_zero = 0;
	if (_BitScanReverse(&leading_zero, value))
	{
		return 31 - leading_zero;
	}
	else
	{
		// If value is zero, return 32 (undefined result)
		return 32;
	}
}

#elif defined(__GNUC__) || defined(__clang__) // GCC or Clang

uint32_t __inline ctz(uint32_t value) // Count trailing zeros
{
	if (value == 0)
		return 32; // If value is zero, return 32
	return __builtin_ctz(value); // Use GCC/Clang built-in
}

uint32_t __inline clz(uint32_t value) // Count leading zeros
{
	if (value == 0)
		return 32; // If value is zero, return 32
	return __builtin_clz(value); // Use GCC/Clang built-in
}

#else
#error Unsupported compiler
#endif

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
#define UNROLL_LOOP _Pragma("loop(unroll)")
#define VECTORIZE_LOOP _Pragma("loop(ivdep)")
#elif defined(__clang__)
#define UNROLL_LOOP _Pragma("clang loop unroll(enable)")
#define VECTORIZE_LOOP _Pragma("clang loop vectorize(enable)")
#elif defined(__GNUC__)
#define UNROLL_LOOP _Pragma("GCC unroll 4")
#define VECTORIZE_LOOP _Pragma("GCC ivdep")
#else
#define UNROLL_LOOP
#define VECTORIZE_LOOP
#endif

#ifndef MATH_PI
#define MATH_PI 3.14159265359
#endif

#define EPSILON 1e-8f
#define WorkingBufferLength (FFTBins * (1 + FFTIntegration / FFTOverlap))


struct iq_balancer_t
{
	float phase;
	float last_phase;

	float amplitude;
	float last_amplitude;

	float iavg;
	float qavg;
	float ibvg;
	float qbvg;

	float integrated_total_power;
	float integrated_image_power;
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

	complex_t* corr;
	complex_t* corr_plus;
	complex_t* working_buffer;
	float* boost;
};

static uint8_t __lib_initialized = 0;
static complex_t __fft_mem[FFTBins];
static float __fft_window[FFTBins];
static float __boost_window[FFTBins];
static complex_t twiddle_factors[FFTBins];
unsigned int bit_reversed[FFTBins]; // Precompute this once for N

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

	//logit window
	/*
	const int odd = FFTBins % 2;
	const int halfsize = FFTBins / 2 + odd;
	__fft_window[0] = 0.0f;
	for (i = halfsize - 2; i > 0; i--)
	{
		__fft_window[i] = (float)i / (float)(halfsize - 1);
		__fft_window[i] /= 1 - __fft_window[i];
		__fft_window[i] = logf(__fft_window[i]);
	}
	float max_val = __fft_window[halfsize - 2] * 2 - __fft_window[halfsize - 3];
	__fft_window[halfsize - 1] = max_val;

	for (i = halfsize - 1; i > 0; i--)
	{
		__fft_window[i] = (__fft_window[i] + max_val) / (max_val * 2);
	}
	for (i = halfsize; i < length; i++)
	{
		__fft_window[i] = __fft_window[FFTBins - i - 1];
	}
	*/



	__lib_initialized = 1;
}

static void window(complex_t* RESTRICT buffer, int length)
{
	int i;

	VECTORIZE_LOOP
		UNROLL_LOOP
		for (i = 0; i < length; i++)
		{
			buffer[i].re *= __fft_window[i];
			buffer[i].im *= __fft_window[i];
		}
}

// Binary-inversion function for sorting data into a frame
unsigned short int reversebits(unsigned int value, unsigned int bits) {
	unsigned int n, result = 0;
	for (n = 0; n < bits; n++) {
		result <<= 1;
		result |= (value & 1);
		value >>= 1;
	}
	return result;
}

// Function for getting table of complex exponents
static void FFT_ExpCalculation(complex_t* twiddle_factors, unsigned int N) {
	float pi = MATH_PI;
	unsigned int step = N;
	unsigned int i = 0;
	while (step >= 2) {
		for (unsigned int k = 0; k < step / 2; k++) {
			float arg = 2 * pi * k / step;
			twiddle_factors[i].re = cos(arg);
			twiddle_factors[i].im = -sin(arg);
			i++;
		}
		step /= 2;
	}
}

void fft(complex_t* buffer, unsigned int N) {
	unsigned int step = N;
	unsigned int i = 0;

	// Bit-reversal sorting
	UNROLL_LOOP
		for (i = 0; i < N; i++) {
			unsigned int reversed = bit_reversed[i];
			if (i < reversed) {
				complex_t temp = buffer[i];
				buffer[i] = buffer[reversed];
				buffer[reversed] = temp;
			}
		}

	// FFT computation
		// FFT computation
	unsigned int twiddle_index = 0;
	while (step > 1) {
		unsigned int half_step = step >> 1; //divide by two
		for (unsigned int k = 0; k < half_step; k++) {
			complex_t twiddle = twiddle_factors[twiddle_index++];
			for (unsigned int n = 0; n < N; n += step) {
				unsigned int even_index = n + k;
				unsigned int odd_index = even_index + half_step;

				complex_t even = buffer[even_index];
				complex_t odd = buffer[odd_index];
				complex_t t;

				t.re = odd.re * twiddle.re - odd.im * twiddle.im;
				t.im = odd.re * twiddle.im + odd.im * twiddle.re;

				buffer[odd_index].re = even.re - t.re;
				buffer[odd_index].im = even.im - t.im;
				buffer[even_index].re = even.re + t.re;
				buffer[even_index].im = even.im + t.im;
			}
		}
		step = half_step;
	}
}



static void cancel_dc(struct iq_balancer_t* iq_balancer, complex_t* RESTRICT iq, int length, uint8_t eval)
{
	int i;
	const float alpha = 0.001f;
	float iavg = iq_balancer->iavg;
	float qavg = iq_balancer->qavg;
	if (!eval)
	{
		for (i = 0; i < length; i++)
		{
			iq[i].re -= iavg;
			iq[i].im -= qavg;
		}
	}
	else {
		UNROLL_LOOP
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

}

static void adjust_benchmark_no_sum(struct iq_balancer_t* iq_balancer, complex_t* RESTRICT iq, float phase, float amplitude)
{
	int i;

	UNROLL_LOOP
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
	float sum = 0;

	UNROLL_LOOP
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
	return sum;
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
	//experimental refactor to provide better vectorization and unrolling
	complex_t cc;
	complex_t* fftPtr = __fft_mem;

	int n, m;
	int i, j;
	int count = 0;
	float power = 0.0f;
	float phase = iq_balancer->phase + step * PhaseStep;
	float amplitude = iq_balancer->amplitude + step * AmplitudeStep;
	if (step == 0)
	{
		for (n = 0, m = 0; n <= length - FFTBins && m < iq_balancer->fft_integration; n += FFTBins / iq_balancer->fft_overlap, m++)
		{
			memcpy(fftPtr, iq + n, FFTBins * sizeof(complex_t));
			power = adjust_benchmark_return_sum(iq_balancer, fftPtr, phase, amplitude);
			if (power <= MinimumPower)
			{
					iq_balancer->power_flag[m] = 0;
					iq_balancer->integrated_total_power += power;
			}
			else
			{
				iq_balancer->power_flag[m] = 1;
				iq_balancer->integrated_total_power += power;
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
				if (iq_balancer->optimal_bin == FFTBins / 2) {
					for (i = EdgeBinsToSkip; i <= FFTBins - EdgeBinsToSkip; i++)
					{
						power = fftPtr[i].re * fftPtr[i].re + fftPtr[i].im * fftPtr[i].im;
						iq_balancer->boost[i] += power;
						iq_balancer->integrated_image_power += power;


					}
				}
				else {
					for (i = EdgeBinsToSkip; i <= FFTBins - EdgeBinsToSkip; i++)
					{
						power = fftPtr[i].re * fftPtr[i].re + fftPtr[i].im * fftPtr[i].im;
						iq_balancer->boost[i] += power;
						iq_balancer->integrated_image_power += power * __boost_window[abs(FFTBins - i - iq_balancer->optimal_bin)];
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
	float invskip = 1.0f / EdgeBinsToSkip;
	complex_t acc = { 0, 0 };
	for (i = EdgeBinsToSkip, j = FFTBins - EdgeBinsToSkip; i <= FFTBins - EdgeBinsToSkip; i++, j--)
	{
		int distance = abs(i - FFTBins / 2);
		if (distance > CenterBinsToSkip)
		{
			float weight = (distance > EdgeBinsToSkip) ? 1.0f : (distance * invskip);
			if (iq_balancer->optimal_bin != FFTBins / 2)
			{
				weight *= __boost_window[abs(iq_balancer->optimal_bin - i)];
			}
			weight *= iq_balancer->boost[j] / (iq_balancer->boost[i] + EPSILON);
			acc.re += ccorr[i].re * weight;
			acc.im += ccorr[i].im * weight;
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

	iq_balancer->phase = phase;
	iq_balancer->amplitude = amplitude;
}


static void adjust_phase_amplitude(struct iq_balancer_t* iq_balancer, complex_t* RESTRICT iq, int length)
{
	int i;
	float scale = 1.0f / (length - 1);

	VECTORIZE_LOOP
		UNROLL_LOOP
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
#define MIN(a,b) ((a) < (b) ? (a) : (b))

void ADDCALL iq_balancer_process(struct iq_balancer_t* iq_balancer, complex_t* RESTRICT  iq, int length, bool eval)
{
	int count;
	cancel_dc(iq_balancer, iq, length, eval);


	if (eval)
	{
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
	FFT_ExpCalculation(twiddle_factors, FFTBins);
	for (unsigned int i = 0; i < FFTBins; i++) {
		bit_reversed[i] = reversebits(i, ctz(FFTBins));
	}
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
