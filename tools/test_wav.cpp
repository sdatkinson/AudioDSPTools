#include <array>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "dsp/wav.h"

namespace
{
std::string ToUTF8(const std::filesystem::path& path)
{
  const auto utf8 = path.u8string();
  return {utf8.begin(), utf8.end()};
}
} // namespace

int main()
{
#ifdef _WIN32
  const auto emojiDirectoryName = std::filesystem::path(L"nam-\U0001F3B8");
#else
  const auto emojiDirectoryName = std::filesystem::path("nam-\xF0\x9F\x8E\xB8");
#endif
  const auto testDirectory = std::filesystem::temp_directory_path() / emojiDirectoryName;
  const auto wavPath = testDirectory / "ir.wav";

  std::filesystem::create_directories(testDirectory);

  // 16-bit, mono, 48 kHz PCM WAV containing one silent sample.
  constexpr std::array<unsigned char, 46> wavData{
    'R', 'I',  'F',  'F', 38, 0, 0,    0, 'W', 'A', 'V', 'E', 'f', 'm', 't', ' ', 16,  0, 0, 0, 1, 0, 1,
    0,   0x80, 0xBB, 0,   0,  0, 0x77, 1, 0,   2,   0,   16,  0,   'd', 'a', 't', 'a', 2, 0, 0, 0, 0, 0};

  {
    std::ofstream wavFile(wavPath, std::ios::binary);
    wavFile.write(reinterpret_cast<const char*>(wavData.data()), wavData.size());
  }

  std::vector<float> audio;
  double sampleRate = 0.0;
  const auto utf8Path = ToUTF8(wavPath);
  const auto result = dsp::wav::Load(utf8Path.c_str(), audio, sampleRate);

  std::filesystem::remove_all(testDirectory);

  if (result != dsp::wav::LoadReturnCode::SUCCESS || audio.size() != 1 || sampleRate != 48000.0)
  {
    std::cerr << "Failed to load WAV from UTF-8 path" << std::endl;
    return 1;
  }

  return 0;
}
