//
//  ImpulseResponse.cpp
//  NeuralAmpModeler-macOS
//
//  Created by Steven Atkinson on 12/30/22.
//

#include "Resample.h"
#include "wav.h"

#include "ImpulseResponse.h"

dsp::ImpulseResponse::ImpulseResponse(const char* fileName, const double sampleRate)
: mWavState(dsp::wav::LoadReturnCode::ERROR_OTHER)
, mSampleRate(sampleRate)
{
  // Try to load the WAV
  size_t channels = 0;
  std::vector<float> interleaved;
  this->mWavState = dsp::wav::Load(fileName, interleaved, this->mRawAudioSampleRate, channels);
  if (this->mWavState == dsp::wav::LoadReturnCode::SUCCESS)
  {
    for (size_t i = 0; i < interleaved.size(); i += channels)
    {
      mRawAudio.push_back(interleaved[i]);
      if (channels == 2)
        mRawAudioRight.push_back(interleaved[i + 1]);
    }
  }
  if (this->mWavState != dsp::wav::LoadReturnCode::SUCCESS)
  {
    std::stringstream ss;
    ss << "Failed to load IR at " << fileName << std::endl;
  }
  else
    // Set the weights based on the raw audio.
    this->_SetWeights();
}

dsp::ImpulseResponse::ImpulseResponse(const IRData& irData, const double sampleRate)
: mWavState(dsp::wav::LoadReturnCode::SUCCESS)
, mSampleRate(sampleRate)
{
  this->mRawAudio = irData.mRawAudio;
  this->mRawAudioRight = irData.mRawAudioRight;
  this->mRawAudioSampleRate = irData.mRawAudioSampleRate;
  this->_SetWeights();
}

double** dsp::ImpulseResponse::Process(double** inputs, const size_t numChannels, const size_t numFrames)
{
  if (numChannels == 0 || mWeight.size() == 0)
    throw std::runtime_error("IR processing requires input and a loaded IR");
  const auto outputChannels = std::max(numChannels, GetNumIRChannels());
  this->_PrepareBuffers(outputChannels, numFrames);
  this->_UpdateHistory(inputs, numChannels, numFrames);

  for (size_t i = 0, j = this->mHistoryIndex - this->mHistoryRequired; i < numFrames; i++, j++)
  {
    auto input = Eigen::Map<const Eigen::VectorXf>(&this->mHistory[j], this->mHistoryRequired + 1);
    this->mOutputs[0][i] = (double)this->mWeight.dot(input);
    if (GetNumIRChannels() == 2)
      this->mOutputs[1][i] = (double)this->mWeightRight.dot(input);
  }
  // Copy out for more-than-mono.
  for (size_t c = GetNumIRChannels(); c < outputChannels; c++)
    for (size_t i = 0; i < numFrames; i++)
      this->mOutputs[c][i] = this->mOutputs[0][i];

  this->_AdvanceHistoryIndex(numFrames);
  return this->_GetPointers();
}

void dsp::ImpulseResponse::Reset(size_t maxFrames, size_t numOutputChannels)
{
  _PrepareBuffers(std::max(numOutputChannels, GetNumIRChannels()), maxFrames);
  _EnsureHistorySize(std::max(size_t{1}, maxFrames));
  std::fill(mHistory.begin(), mHistory.end(), 0.0f);
  mHistoryIndex = mHistoryRequired;
}

void dsp::ImpulseResponse::_SetWeights()
{
  if (!std::isfinite(mSampleRate) || mSampleRate <= 0.0
      || !std::isfinite(mRawAudioSampleRate) || mRawAudioSampleRate <= 0.0
      || mRawAudio.empty()
      || (!mRawAudioRight.empty() && mRawAudioRight.size() != mRawAudio.size()))
    throw std::runtime_error("Invalid IR samples or sample rate");
  for (const auto* channel : { &mRawAudio, &mRawAudioRight })
    for (const auto sample : *channel)
      if (!std::isfinite(sample))
        throw std::runtime_error("Non-finite IR sample");
  const auto makeWeights = [this](const std::vector<float>& rawAudio) {
    std::vector<float> resampled;
    if (mRawAudioSampleRate == mSampleRate)
      resampled = rawAudio;
    else
    {
      std::vector<float> padded(rawAudio.size() + 2, 0.0f);
      std::copy(rawAudio.begin(), rawAudio.end(), padded.begin() + 1);
      dsp::ResampleCubic<float>(padded, mRawAudioSampleRate, mSampleRate, 0.0, resampled);
    }
    const size_t irLength = std::min(resampled.size(), mMaxLength);
    if (irLength == 0)
      throw std::runtime_error("Empty resampled IR");
    Eigen::VectorXf weights(irLength);
    // Preserve the existing -18 dB and sample-rate-dependent gain law.
    const float gain = pow(10, -18 * 0.05) * 48000 / mSampleRate;
    for (size_t i = 0; i < irLength; ++i)
      weights[irLength - 1 - i] = gain * resampled[i];
    return weights;
  };
  mWeight = makeWeights(mRawAudio);
  if (!mRawAudioRight.empty())
    mWeightRight = makeWeights(mRawAudioRight);
  mHistoryRequired = static_cast<size_t>(mWeight.size()) - 1;
}

dsp::ImpulseResponse::IRData dsp::ImpulseResponse::GetData()
{
  IRData irData;
  irData.mRawAudio = this->mRawAudio;
  irData.mRawAudioRight = this->mRawAudioRight;
  irData.mRawAudioSampleRate = this->mRawAudioSampleRate;
  return irData;
}
