/*
Copyright (c) 2016-2023, Youssef Touil <youssef@airspy.com>
Copyright (c) 2018, Leif Asbrink <leif@sm5bsz.com>

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

#ifndef __IQ_BALANCER_H__
#define __IQ_BALANCER_H__


#include "airspyhf.h"

#define FFTBins (4 * 1024)
#define BoostFactor 100000.0
#define BinsToOptimize (FFTBins/25)
#define EdgeBinsToSkip (FFTBins/22)
#define CenterBinsToSkip 2
#define MaxLookback 4 
#define PhaseGradStep 0.414213562373f  //tan(pi/2*MaxLookBack)  # Maximum rotation before correlation loss across 4 samples (MaxLookback)

#define AmplitudeGradStep 0.9636633660515027f//(1 - tan(pi/8 - arctan(phase_step))) / (1 + tan(pi/8 - arctan(phase_step))) 
#define PhaseStep AmplitudeGradStep
#define AmplitudeStep PhaseGradStep
#define MaxMu 50.0f
#define MinDeltaMu 1e-5f
#define MinimumPower 1e-5f
#define PowerThreshold 0.5f
#define BuffersToSkipOnReset 2
#define MaxPowerDecay 0.98f
#define MaxPowerRatio 0.8f
#define BoostWindowNorm (MaxPowerRatio / 95)
#define EPSILON 0.00000762939f //~sqrt(4096) * eps

#if defined(__arm__) && !defined(__force_hiq__)
#define BuffersToSkip 4
#define FFTIntegration 2
#define FFTOverlap 1
#define CorrelationIntegration 4
#else
#define BuffersToSkip 1
#define FFTIntegration 8
#define FFTOverlap 4
#define CorrelationIntegration 32
#endif

struct iq_balancer_t;

typedef airspyhf_complex_float_t complex_t;


ADDAPI struct iq_balancer_t* ADDCALL iq_balancer_create(float initial_phase, float initial_amplitude);
ADDAPI void ADDCALL iq_balancer_set_optimal_point(struct iq_balancer_t* iq_balancer, float w);
ADDAPI void ADDCALL iq_balancer_configure(struct iq_balancer_t* iq_balancer, int buffers_to_skip, int fft_integration, int fft_overlap, int correlation_integration);
ADDAPI void ADDCALL iq_balancer_process(struct iq_balancer_t* iq_balancer, complex_t*  iq, int length);
ADDAPI void ADDCALL iq_balancer_destroy(struct iq_balancer_t* iq_balancer);
#endif
