/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#ifndef _MOTOR_CRC_H_
#define MOTOR_CRC_H_

#include <array>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/bms_cmd.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"

constexpr int HIGHLEVEL = 0xee;
constexpr int LOWLEVEL = 0xff;
constexpr int TRIGERLEVEL = 0xf0;
constexpr double POS_STOP_F = (2.146E+9F);
constexpr double VEL_STOP_F = (16000.0F);

// joint index
constexpr int FR_0 = 0;
constexpr int FR_1 = 1;
constexpr int FR_2 = 2;

constexpr int FL_0 = 3;
constexpr int FL_1 = 4;
constexpr int FL_2 = 5;

constexpr int RR_0 = 6;
constexpr int RR_1 = 7;
constexpr int RR_2 = 8;

constexpr int RL_0 = 9;
constexpr int RL_1 = 10;
constexpr int RL_2 = 11;

using BmsCmd = struct {
  uint8_t off;  // off 0xA5
  std::array<uint8_t, 3> reserve;
};

using MotorCmd = struct {
  uint8_t mode;  // desired working mode
  float q;       // desired angle (unit: radian)
  float dq;      // desired velocity (unit: radian/second)
  float tau;     // desired output torque (unit: N.m)
  float Kp;      // desired position stiffness (unit: N.m/rad )
  float Kd;      // desired velocity stiffness (unit: N.m/(rad/s) )
  std::array<uint32_t, 3> reserve;
};  // motor control

using LowCmd = struct {
  std::array<uint8_t, 2> head;
  uint8_t levelFlag;
  uint8_t frameReserve;

  std::array<uint32_t, 2> SN;
  std::array<uint32_t, 2> version;
  uint16_t bandWidth;
  std::array<MotorCmd, 20> motorCmd;
  BmsCmd bms;
  std::array<uint8_t, 40> wirelessRemote;
  std::array<uint8_t, 12> led;
  std::array<uint8_t, 2> fan;
  uint8_t gpio;
  uint32_t reserve;

  uint32_t crc;
};

void get_crc(unitree_go::msg::LowCmd& msg);

#endif