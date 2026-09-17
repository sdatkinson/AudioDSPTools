//
//  wav.h
//  NeuralAmpModeler-macOS
//
//  Created by Steven Atkinson on 12/31/22.
//

#pragma once

#include <fstream>
#include <string>
#include <vector>

namespace dsp
{
namespace wav
{
enum class LoadReturnCode
{
  SUCCESS = 0,
  ERROR_OPENING,
  ERROR_NOT_RIFF,
  ERROR_NOT_WAVE,
  ERROR_MISSING_FMT,
  ERROR_INVALID_FILE,
  ERROR_UNSUPPORTED_FORMAT_ALAW,
  ERROR_UNSUPPORTED_FORMAT_MULAW,
  ERROR_UNSUPPORTED_FORMAT_OTHER,
  ERROR_UNSUPPORTED_BITS_PER_SAMPLE,
  ERROR_OTHER,
  ERROR_UNSUPPORTED_CHANNEL_COUNT
};

// Get a string describing the error
std::string GetMsgForLoadReturnCode(LoadReturnCode rc);

// Load mono or stereo WAV samples. On success, audio contains interleaved
// samples (L0, R0, L1, R1, ... for stereo), sampleRate is the source rate in Hz,
// and numChannels is 1 or 2. Mono samples remain sequential.
//
// This replaces the three-argument, mono-only API: callers must supply a size_t
// channel-count output and handle interleaved stereo data. ERROR_NOT_MONO has
// been removed; unsupported channel counts return ERROR_UNSUPPORTED_CHANNEL_COUNT.
LoadReturnCode Load(const char* fileName, std::vector<float>& audio, double& sampleRate, size_t& numChannels);

// Load samples, 16-bit
void _LoadSamples16(std::ifstream& wavFile, const int chunkSize, std::vector<float>& samples);
// Load samples, 24-bit
void _LoadSamples24(std::ifstream& wavFile, const int chunkSize, std::vector<float>& samples);
// Load samples, 32-bit
void _LoadSamples32FloatingPoint(std::ifstream& wavFile, const int chunkSize, std::vector<float>& samples);
// Load samples, 32-bit fixed point
void _LoadSamples32FixedPoint(std::ifstream& wavFile, const int chunkSize, std::vector<float>& samples);

// Read in a 24-bit sample and convert it to an int
int _ReadSigned24BitInt(std::ifstream& stream);
}; // namespace wav
}; // namespace dsp
