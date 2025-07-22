#pragma once
#include <atomic>
#include <unordered_map>

namespace unitree::ros2 {

struct CliConfig {
  uint32_t timeout;
};

class ClientConfig {
 public:
  ClientConfig() : service_ready_(false) {}
  virtual ~ClientConfig() = default;

  void SetApiTimeout(int32_t api_id, uint32_t timeout);
  uint32_t GetApiTimeout(int32_t api_id);

  bool IsServiceReady() { return service_ready_; }

 protected:
  void SetServiceStatus(bool ready) { service_ready_ = ready; }

 private:
  bool GetConfig(int32_t api_id, CliConfig &config);

  std::unordered_map<int32_t, CliConfig> config_map_;

  std::atomic_bool service_ready_;
};
}  // namespace unitree::ros2
