#include "client/g1/g1_loco_client.hpp"

using namespace unitree::ros2::g1;

#define LOCO_SERVICE_NAME "sport"
#define SEND_LOCO_REQUEST(REQUEST_FUNC, ...) \
  SEND_REQUEST(param_, REQUEST_FUNC, __VA_ARGS__)
#define PARSE_LOCO_RESPONSE(RESPONSE_FUNC, ...) \
  PARSE_RESPONSE(param_, RESPONSE_FUNC, __VA_ARGS__)

LocoClient::LocoClient(const std::string& node_name)
    : BaseClient(node_name, LOCO_SERVICE_NAME) {}

int32_t LocoClient::GetFsmId(int32_t& fsm_id){
    SEND_LOCO_REQUEST(GetFsmIdReq) PARSE_LOCO_RESPONSE(GetFsmIdRes, fsm_id)}

int32_t LocoClient::GetFsmMode(int32_t& fsm_mode){
    SEND_LOCO_REQUEST(GetFsmModeReq)
        PARSE_LOCO_RESPONSE(GetFsmModeRes, fsm_mode)}

int32_t LocoClient::GetBalanceMode(int32_t& balance_mode){
    SEND_LOCO_REQUEST(GetBalanceModeReq)
        PARSE_LOCO_RESPONSE(GetBalanceModeRes, balance_mode)}

int32_t LocoClient::GetSwingHeight(float& swing_height){
    SEND_LOCO_REQUEST(GetSwingHeightReq)
        PARSE_LOCO_RESPONSE(GetSwingHeightRes, swing_height)}

int32_t LocoClient::GetStandHeight(float& stand_height){
    SEND_LOCO_REQUEST(GetStandHeightReq)
        PARSE_LOCO_RESPONSE(GetStandHeightRes, stand_height)}

int32_t LocoClient::GetPhase(std::vector<float>& phase){
    SEND_LOCO_REQUEST(GetPhaseReq) PARSE_LOCO_RESPONSE(GetPhaseRes, phase)}

int32_t LocoClient::SetFsmId(int32_t fsm_id){
    SEND_LOCO_REQUEST(SetFsmIdReq, fsm_id) PARSE_LOCO_RESPONSE(SetFsmIdRes)}

int32_t LocoClient::SetBalanceMode(int32_t balance_mode){
    SEND_LOCO_REQUEST(SetBalanceModeReq, balance_mode)
        PARSE_LOCO_RESPONSE(SetBalanceModeRes)}

int32_t LocoClient::SetSwingHeight(float swing_height){
    SEND_LOCO_REQUEST(SetSwingHeightReq, swing_height)
        PARSE_LOCO_RESPONSE(SetSwingHeightRes)}

int32_t LocoClient::SetStandHeight(float stand_height){
    SEND_LOCO_REQUEST(SetStandHeightReq, stand_height)
        PARSE_LOCO_RESPONSE(SetStandHeightRes)}

int32_t LocoClient::SetVelocity(float vx, float vy, float omega,
                                float duration){
    SEND_LOCO_REQUEST(SetVelocityReq, vx, vy, omega, duration)
        PARSE_LOCO_RESPONSE(SetVelocityRes)}

int32_t LocoClient::SetTaskId(int32_t task_id){
    SEND_LOCO_REQUEST(SetTaskIdReq, task_id) PARSE_LOCO_RESPONSE(SetTaskIdRes)}

int32_t LocoClient::Damp() {
  return SetFsmId(1);
}

int32_t LocoClient::Start() { return SetFsmId(200); }

int32_t LocoClient::Squat() { return SetFsmId(2); }

int32_t LocoClient::Sit() { return SetFsmId(3); }

int32_t LocoClient::StandUp() { return SetFsmId(4); }

int32_t LocoClient::ZeroTorque() { return SetFsmId(0); }

int32_t LocoClient::StopMove() { return SetVelocity(0.F, 0.F, 0.F); }

int32_t LocoClient::HighStand() {
  return SetStandHeight(std::numeric_limits<uint32_t>::max());
}

int32_t LocoClient::LowStand() {
  return SetStandHeight(std::numeric_limits<uint32_t>::min());
}

int32_t LocoClient::Move(float vx, float vy, float vyaw, bool continous_move) {
  return SetVelocity(vx, vy, vyaw, continous_move ? 864000.F : 1.F);
}

int32_t LocoClient::Move(float vx, float vy, float vyaw) {
  return Move(vx, vy, vyaw, continous_move_);
}

int32_t LocoClient::SwitchMoveMode(bool flag) {
  continous_move_ = flag;
  return 0;
}

int32_t LocoClient::BalanceStand() { return SetBalanceMode(0); }

int32_t LocoClient::ContinuousGait(bool flag) {
  return SetBalanceMode(flag ? 1 : 0);
}

int32_t LocoClient::WaveHand(bool turn_flag) {
  return SetTaskId(turn_flag ? 1 : 0);
}

int32_t LocoClient::ShakeHand(int stage) {
  switch (stage) {
    case 0:
      first_shake_hand_stage_ = false;
      return SetTaskId(2);
    case 1:
      first_shake_hand_stage_ = true;
      return SetTaskId(3);
    default:
      first_shake_hand_stage_ = !first_shake_hand_stage_;
      return SetTaskId(first_shake_hand_stage_ ? 3 : 2);
  }
}
