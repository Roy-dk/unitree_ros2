#include "client/motion_switch_client.hpp"

using namespace unitree::ros2;

#define MOTIONSWITCHER_SERVICE_NAME "motion_switcher"
#define SEND_MOTIONSWITCH_REQUEST(REQUEST_FUNC, ...) \
  SEND_REQUEST(param_, REQUEST_FUNC, __VA_ARGS__)
#define PARSE_MOTIONSWITCH_RESPONSE(RESPONSE_FUNC, ...) \
  PARSE_RESPONSE(param_, RESPONSE_FUNC, __VA_ARGS__)

MotionSwitchClient::MotionSwitchClient(const std::string &node_name)
    : BaseClient(node_name, MOTIONSWITCHER_SERVICE_NAME) {}

int32_t MotionSwitchClient::CheckMode(std::string &form, std::string &name){
    SEND_MOTIONSWITCH_REQUEST(CheckModeReq)
        PARSE_MOTIONSWITCH_RESPONSE(CheckModeRes, form, name)}

int32_t MotionSwitchClient::SelectMode(const std::string &name_or_alias){
    SEND_MOTIONSWITCH_REQUEST(SelectModeReq, name_or_alias)
        PARSE_MOTIONSWITCH_RESPONSE(SelectModeRes)}

int32_t MotionSwitchClient::ReleaseMode(){
    SEND_MOTIONSWITCH_REQUEST(ReleaseModeReq)
        PARSE_MOTIONSWITCH_RESPONSE(ReleaseModeRes)}

int32_t MotionSwitchClient::SetSilent(bool silent){
    SEND_MOTIONSWITCH_REQUEST(SetSilentReq, silent)
        PARSE_MOTIONSWITCH_RESPONSE(SetSilentRes)}

int32_t MotionSwitchClient::GetSilent(bool &silent) {
  SEND_MOTIONSWITCH_REQUEST(GetSilentReq)
  PARSE_MOTIONSWITCH_RESPONSE(GetSilentRes, silent)
}
