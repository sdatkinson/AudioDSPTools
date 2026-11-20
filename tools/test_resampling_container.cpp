#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <type_traits>
#include <vector>

// The resampling headers normally get these constants from the host.
namespace iplug
{
constexpr double PI = 3.14159265358979323846;
}
namespace dsp
{
constexpr int DEFAULT_BLOCK_SIZE = 256;
}

#include "dsp/ResamplingContainer/ResamplingContainer.h"

namespace
{
using StereoResampler = dsp::ResamplingContainer<double, 2, 12>;
static_assert(std::is_same_v<StereoResampler::LanczosResampler, dsp::LanczosResampler<double, 2, 12>>);

template <size_t Channels>
using Audio = std::array<std::vector<double>, Channels>;

template <size_t Inputs, size_t Outputs, typename Resampler, typename Process>
Audio<Outputs> Render(Resampler& resampler, Audio<Inputs>& input, Process process, int maxBlock)
{
  const size_t frames = input[0].size();
  Audio<Outputs> output;
  for (auto& channel : output)
    channel.resize(frames, std::numeric_limits<double>::quiet_NaN());

  for (size_t offset = 0, block = 0; offset < frames; ++block)
  {
    constexpr int blockSizes[]{1, 7, 31, 127, 256};
    const int count = std::min({blockSizes[block % 5], maxBlock, static_cast<int>(frames - offset)});
    std::array<double*, Inputs> inputs;
    std::array<double*, Outputs> outputs;
    for (size_t ch = 0; ch < Inputs; ++ch)
      inputs[ch] = input[ch].data() + offset;
    for (size_t ch = 0; ch < Outputs; ++ch)
      outputs[ch] = output[ch].data() + offset;
    resampler.ProcessBlock(inputs.data(), outputs.data(), count, process);
    offset += count;
  }
  return output;
}

template <size_t Inputs, size_t Outputs>
void TestChannelCounts()
{
  static_assert(Inputs > 0 && Outputs > 0);
  constexpr size_t frames = 2048;
  constexpr size_t referenceChannels = std::max({Inputs, Outputs, size_t{2}});
  Audio<Inputs> input;
  for (size_t ch = 0; ch < Inputs; ++ch)
  {
    input[ch].resize(frames);
    for (size_t i = 0; i < frames; ++i)
      input[ch][i] = std::sin((ch + 1) * 0.13 * i);
  }
  Audio<referenceChannels> paddedInput;
  for (size_t ch = 0; ch < referenceChannels; ++ch)
    paddedInput[ch] = input[std::min(ch, Inputs - 1)];

  const auto process = [](double** in, double** out, int count) {
    for (int frame = 0; frame < count; ++frame)
    {
      // Include every input even when there are fewer outputs.
      double inputSum = 0.0;
      for (size_t ch = 0; ch < Inputs; ++ch)
        inputSum += in[ch][frame];
      const double sharedSignal = 0.25 * inputSum / Inputs;

      for (size_t ch = 0; ch < Outputs; ++ch)
      {
        const double gain = 1.0 - 0.5 * ch / Outputs;
        out[ch][frame] = gain * (in[ch % Inputs][frame] + sharedSignal);
      }
    }
  };
  const auto paddedProcess = [&](double** in, double** out, int count) {
    process(in, out, count);
    for (size_t ch = Outputs; ch < referenceChannels; ++ch)
      std::fill_n(out[ch], count, 0.0);
  };

  dsp::ResamplingContainer<double, Inputs, 12, Outputs> resampler(48000.0);
  dsp::ResamplingContainer<double, referenceChannels, 12> reference(48000.0);
  for (const double hostRate : {44100.0, 96000.0})
    for (const int maxBlock : {256, 7, 7})
    {
      resampler.Reset(hostRate, maxBlock);
      reference.Reset(hostRate, maxBlock);
      if (resampler.GetLatency() <= 0 || resampler.GetLatency() != reference.GetLatency())
        throw std::runtime_error("Changing the channel counts changed the resampling latency");

      const auto actual = Render<Inputs, Outputs>(resampler, input, process, maxBlock);
      const auto expected = Render<referenceChannels, referenceChannels>(reference, paddedInput, paddedProcess, maxBlock);
      for (size_t ch = 0; ch < Outputs; ++ch)
      {
        if (std::none_of(expected[ch].begin(), expected[ch].end(), [](double sample) { return std::abs(sample) > 0.1; }))
          throw std::runtime_error("Expected an audible signal in every output channel");
        for (size_t frame = 0; frame < frames; ++frame)
          if (!(std::abs(actual[ch][frame] - expected[ch][frame]) <= 1.0e-12))
            throw std::runtime_error("Resampling disagrees with the padded reference");
      }
    }
}
} // namespace

int main()
{
  try
  {
    TestChannelCounts<1, 1>();
    TestChannelCounts<1, 2>();
    TestChannelCounts<2, 1>();
    TestChannelCounts<2, 2>();
    TestChannelCounts<2, 3>();
    TestChannelCounts<3, 5>();
    TestChannelCounts<5, 3>();
  }
  catch (const std::exception& error)
  {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
