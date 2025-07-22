#pragma once

#include "unitree_api/srv/generic.hpp"

namespace unitree::ros2::g1 {

constexpr int32_t ROBOT_API_ID_LOCO_GET_FSM_ID = 7001;
constexpr int32_t ROBOT_API_ID_LOCO_GET_FSM_MODE = 7002;
constexpr int32_t ROBOT_API_ID_LOCO_GET_BALANCE_MODE = 7003;
constexpr int32_t ROBOT_API_ID_LOCO_GET_SWING_HEIGHT = 7004;
constexpr int32_t ROBOT_API_ID_LOCO_GET_STAND_HEIGHT = 7005;
constexpr int32_t ROBOT_API_ID_LOCO_GET_PHASE = 7006;  // deprecated

constexpr int32_t ROBOT_API_ID_LOCO_SET_FSM_ID = 7101;
constexpr int32_t ROBOT_API_ID_LOCO_SET_BALANCE_MODE = 7102;
constexpr int32_t ROBOT_API_ID_LOCO_SET_SWING_HEIGHT = 7103;
constexpr int32_t ROBOT_API_ID_LOCO_SET_STAND_HEIGHT = 7104;
constexpr int32_t ROBOT_API_ID_LOCO_SET_VELOCITY = 7105;
constexpr int32_t ROBOT_API_ID_LOCO_SET_ARM_TASK = 7106;

class LocoClientApi {
 public:
  static void GetFsmIdReq(
      const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

  static int32_t GetFsmIdRes(
      const std::shared_ptr<unitree_api::srv::Generic::Response> &res,
      int32_t &fsm_id);

  static void GetFsmModeReq(
      const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

  static int32_t GetFsmModeRes(
      const std::shared_ptr<unitree_api::srv::Generic::Response> &res,
      int32_t &fsm_mode);

  static void GetBalanceModeReq(
      const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

  static int32_t GetBalanceModeRes(
      const std::shared_ptr<unitree_api::srv::Generic::Response> &res,
      int32_t &balance_mode);

  static void GetSwingHeightReq(
      const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

  static int32_t GetSwingHeightRes(
      const std::shared_ptr<unitree_api::srv::Generic::Response> &res,
      float &swing_height);

  static void GetStandHeightReq(
      const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

  static int32_t GetStandHeightRes(
      const std::shared_ptr<unitree_api::srv::Generic::Response> &res,
      float &stand_height);

  static void GetPhaseReq(
      const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

  static int32_t GetPhaseRes(
      const std::shared_ptr<unitree_api::srv::Generic::Response> &res,
      std::vector<float> &phase);

  static void SetFsmIdReq(
      int32_t fsm_id,
      const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

  static int32_t SetFsmIdRes(
      const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

  static void SetBalanceModeReq(
      int32_t balance_mode,
      const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

  static int32_t SetBalanceModeRes(
      const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

  static void SetSwingHeightReq(
      float swing_height,
      const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

  static int32_t SetSwingHeightRes(
      const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

  static void SetStandHeightReq(
      float stand_height,
      const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

  static int32_t SetStandHeightRes(
      const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

  static void SetVelocityReq(
      float vx, float vy, float omega, float duration,
      const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

  static int32_t SetVelocityRes(
      const std::shared_ptr<unitree_api::srv::Generic::Response> &res);

  static void SetTaskIdReq(
      int32_t task_id,
      const std::shared_ptr<unitree_api::srv::Generic::Request> &req);

  static int32_t SetTaskIdRes(
      const std::shared_ptr<unitree_api::srv::Generic::Response> &res);
};

}  // namespace unitree::ros2::g1
