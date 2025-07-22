#include "motor_crc.h"

#include "utils/crc.hpp"

void get_crc(unitree_go::msg::LowCmd& msg) {
  LowCmd raw{};
  memcpy(raw.head.data(), msg.head.data(), 2);

  raw.levelFlag = msg.level_flag;
  raw.frameReserve = msg.frame_reserve;

  memcpy(raw.SN.data(), msg.sn.data(), 8);
  memcpy(raw.version.data(), msg.version.data(), 8);

  raw.bandWidth = msg.bandwidth;

  for (int i = 0; i < 20; i++) {
    raw.motorCmd[i].mode = msg.motor_cmd[i].mode;
    raw.motorCmd[i].q = msg.motor_cmd[i].q;
    raw.motorCmd[i].dq = msg.motor_cmd[i].dq;
    raw.motorCmd[i].tau = msg.motor_cmd[i].tau;
    raw.motorCmd[i].Kp = msg.motor_cmd[i].kp;
    raw.motorCmd[i].Kd = msg.motor_cmd[i].kd;

    memcpy(raw.motorCmd[i].reserve.data(), msg.motor_cmd[i].reserve.data(), 12);
  }

  raw.bms.off = msg.bms_cmd.off;
  memcpy(raw.bms.reserve.data(), msg.bms_cmd.reserve.data(), 3);

  memcpy(raw.wirelessRemote.data(), msg.wireless_remote.data(), 40);

  memcpy(raw.led.data(), msg.led.data(), 12);  // go2
  memcpy(raw.fan.data(), msg.fan.data(), 2);
  raw.gpio = msg.gpio;  // go2

  raw.reserve = msg.reserve;

  raw.crc = unitree::common::crc32_core(reinterpret_cast<uint32_t*>(&raw),
                                        (sizeof(LowCmd) >> 2) - 1);
  msg.crc = raw.crc;
}
