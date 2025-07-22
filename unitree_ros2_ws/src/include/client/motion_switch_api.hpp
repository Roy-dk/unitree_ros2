#pragma once

#include "unitree_api/srv/generic.hpp"

namespace unitree::ros2 {

constexpr int32_t MOTION_SWITCHER_API_ID_CHECK_MODE = 1001;
constexpr int32_t MOTION_SWITCHER_API_ID_SELECT_MODE = 1002;
constexpr int32_t MOTION_SWITCHER_API_ID_RELEASE_MODE = 1003;
constexpr int32_t MOTION_SWITCHER_API_ID_SET_SILENT = 1004;
constexpr int32_t MOTION_SWITCHER_API_ID_GET_SILENT = 1005;

class MotionSwitchApi {
 public:
  static void CheckModeReq(
      const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

  static int32_t CheckModeRes(
      const std::shared_ptr<unitree_api::srv::Generic::Response> &res,
      std::string &form, std::string &name);

  static void SelectModeReq(
      const std::string &name_or_alias,
      const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

  static int32_t SelectModeRes(
      const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

  static void ReleaseModeReq(
      const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

  static int32_t ReleaseModeRes(
      const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

  static void SetSilentReq(
      bool silent,
      const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

  static int32_t SetSilentRes(
      const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

  static void GetSilentReq(
      const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

  static int32_t GetSilentRes(
      const std::shared_ptr<unitree_api::srv::Generic::Response> &res,
      bool &silent);
};
}  // namespace unitree::ros2
