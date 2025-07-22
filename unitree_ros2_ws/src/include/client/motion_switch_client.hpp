#pragma once

#include "client/base_client.hpp"
#include "client/motion_switch_api.hpp"

namespace unitree::ros2 {

class MotionSwitchClient : public BaseClient {
 public:
  explicit MotionSwitchClient(
      const std::string &node_name = "motion_switch_client");

  int32_t CheckMode(std::string &form, std::string &name);

  int32_t SelectMode(const std::string &name_or_alias);

  int32_t ReleaseMode();

  int32_t SetSilent(bool silent);

  int32_t GetSilent(bool &silent);

 private:
  MotionSwitchApi param_;
};

}  // namespace unitree::ros2
