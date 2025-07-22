#include "client/config.hpp"

using namespace unitree::ros2;

constexpr uint32_t DEFAULT_API_TIME_OUT = 10;

void ClientConfig::SetApiTimeout(int32_t api_id, uint32_t timeout) {
  CliConfig config{};
  GetConfig(api_id, config);
  config.timeout = timeout;
  config_map_[api_id] = config;
}

uint32_t unitree::ros2::ClientConfig::GetApiTimeout(int32_t api_id) {
  CliConfig config{};
  if (GetConfig(api_id, config)) {
    return config.timeout;
  }
  return DEFAULT_API_TIME_OUT;
}

bool ClientConfig::GetConfig(int32_t api_id, CliConfig &config) {
  if (config_map_.count(api_id) == 0) {
    return false;
  }
  config = config_map_[api_id];

  return true;
}