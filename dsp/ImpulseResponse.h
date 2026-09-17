//
//  ImpulseResponse.h
//  NeuralAmpModeler-macOS
//
//  Created by Steven Atkinson on 12/30/22.
//
// Impulse response processing

#pragma once

#include <filesystem>

#include <Eigen/Dense>

#include "dsp.h"
#include "wav.h"

namespace dsp
{
// Mono-input convolution with a mono or stereo IR, loaded from a WAV or IRData.
// Each IR channel uses cubic resampling, an 8192-tap limit, and the gain law
// pow(10.0, -18.0 / 20.0) * 48000 / sampleRate, where sampleRate is the processing rate.
class ImpulseResponse : public History
{
public:
  struct IRData;
  ImpulseResponse(const char* fileName, const double sampleRate);
  ImpulseResponse(const IRData& irData, const double sampleRate);
  // Only inputs[0] is read; independent stereo inputs are not processed.
  // Call Process(inputs, 1, frames) for mono input. The returned buffer contains
  // max(numChannels, GetNumIRChannels()) outputs, with stereo IR channels ordered
  // left then right. A mono IR is duplicated to all requested outputs.
  double** Process(double** inputs, const size_t numChannels, const size_t numFrames) override;
  // Return both original IR channels and their source sample rate, before
  // resampling, truncation, or gain adjustment, for storage or reconstruction.
  IRData GetData();
  // Number of channels in the IR: one for mono, two for stereo.
  size_t GetNumIRChannels() const { return mRawAudioRight.empty() ? 1 : 2; }
  // Call off the audio thread to reserve processing storage and clear history.
  // numOutputChannels should match numChannels in subsequent Process calls.
  // Blocks up to maxFrames reuse that storage, including variable block sizes.
  void Reset(size_t maxFrames, size_t numOutputChannels = 2);
  double GetSampleRate() const { return mSampleRate; };
  // TODO states for the IR class
  dsp::wav::LoadReturnCode GetWavState() const { return this->mWavState; };

private:
  // Set the weights, given that the plugin is running at the provided sample
  // rate.
  void _SetWeights();

  // State of audio
  dsp::wav::LoadReturnCode mWavState;
  // Keep a copy of the raw audio that was loaded so that it can be resampled
  std::vector<float> mRawAudio;
  std::vector<float> mRawAudioRight;
  double mRawAudioSampleRate;
  double mSampleRate;

  const size_t mMaxLength = 8192;
  // The weights
  Eigen::VectorXf mWeight;
  Eigen::VectorXf mWeightRight;
};

struct dsp::ImpulseResponse::IRData
{
  // Original mono/left samples, before resampling or gain adjustment.
  std::vector<float> mRawAudio;
  // Source sample rate in Hz.
  double mRawAudioSampleRate;
  // Original right samples. Leave empty for mono behavior; stereo IRs require
  // the same number of frames as mRawAudio.
  std::vector<float> mRawAudioRight;
};

}; // namespace dsp
