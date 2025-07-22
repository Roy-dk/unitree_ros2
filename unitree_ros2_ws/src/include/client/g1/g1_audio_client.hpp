#pragma once

#include "client/base_client.hpp"
#include "client/g1/g1_audio_api.hpp"

namespace unitree::ros2::g1 {

class AudioClient : public BaseClient {
 public:
  explicit AudioClient(const std::string &node_name = "g1_voice_lient");

  int32_t TtsMaker(const std::string &text, int32_t speaker_id);

  int32_t GetVolume(uint8_t &volume);

  int32_t SetVolume(uint8_t volume);

  int32_t PlayStream(const std::string &app_name, const std::string &stream_id,
                     const std::vector<uint8_t> &pcm_data);

  int32_t PlayStop(const std::string &app_name);

  int32_t LedControl(uint8_t r, uint8_t g, uint8_t b);

 private:
  AudioClientApi param_;
};

}  // namespace unitree::ros2::g1
