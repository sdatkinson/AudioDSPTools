// File: ResamplingContainer.h
// Created Date: Monday February 5th 2024
// Author: Mikko Honkala (mikko.honkala@gmail.com)

// A container for real-time resampling using a Low pass filtered Lanczos anti-aliasing filter

#pragma once

#include <iostream>
#include <functional>
#include <cmath>

#include "LanczosResampler.h"
#include "AudioDSPTools/dsp/RecursiveLinearFilter.h"


namespace dsp
{
/**
 * Extends the LanczosResampler class by integrating a LowPassBiquad filter to reduce aliasing in
 * the case of downsampilng.
 * This subclass applies a low-pass filter to the input audio signal before performing
 * the Lanczos downsampling process. In case of upsamping, the filter is not used.
 *
 * Template parameters:
 * - T: The data type of the audio samples (e.g., float or double).
 * - NCHANS: The number of audio channels to process (e.g., 1 for mono, 2 for stereo).
 * - A: The filter size parameter of the Lanczos resampler, affecting quality and latency.
 */
template<typename T = double, int NCHANS = 2, size_t A = 12>
class LanczosResamplerWithLPF : public LanczosResampler<T, NCHANS, A> {
public:
    recursive_linear_filter::LowPassBiquad lowPassFilter;
    bool applyLPF = false; // Conditionally apply LPF

    // Constructor
    LanczosResamplerWithLPF(float inputRate, float outputRate, float cutoffRatioOfNyquist = 0.9)
    : LanczosResampler<T, NCHANS, A>(inputRate, outputRate) {
        // Compute cutoff frequency based on the output rate, set to just below Nyquist
        float cutoffFrequency = outputRate / 2.0 * cutoffRatioOfNyquist; // e.g., 90% of Nyquist frequency

        // Only initialize and apply LPF if downsampling
        if (outputRate < inputRate) {
            double qualityFactor = 0.707; // Common choice for a Butterworth filter
            double gainDB = 0.0; // No gain change for a low-pass filter
            recursive_linear_filter::BiquadParams params(inputRate, cutoffFrequency, qualityFactor, gainDB);
            lowPassFilter.SetParams(params);
            applyLPF = true;
        }
    }

    // Override the PushBlock method to conditionally apply LPF
    void PushBlock(T** inputs, size_t nFrames) override {
        if (applyLPF) {
            // Apply the low-pass filter to the inputs before resampling
            DSP_SAMPLE** dspInputs = reinterpret_cast<DSP_SAMPLE**>(inputs);
            DSP_SAMPLE** filteredOutputs = lowPassFilter.Process(dspInputs, NCHANS, nFrames);
            LanczosResampler<T, NCHANS, A>::PushBlock(reinterpret_cast<T**>(filteredOutputs), nFrames);
        } else {
            // Directly call base class PushBlock without LPF
            LanczosResampler<T, NCHANS, A>::PushBlock(inputs, nFrames);
        }
    }
};

}; // namespace dsp
