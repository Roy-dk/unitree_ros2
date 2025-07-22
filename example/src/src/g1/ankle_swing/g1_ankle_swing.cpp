/**
 * This example demonstrates how to use ROS2 to control ankle commands of
 *unitree g1 robot
 **/
#include <iomanip>

#include "client/motion_switch_client.hpp"
#include "motor_crc_hg.h"
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "utils/gamepad.hpp"
#include "utils/utils.hpp"

using namespace unitree::common;
using std::placeholders::_1;

static const std::string HG_CMD_TOPIC = "lowcmd";
static const std::string HG_IMU_TORSO = "secondary_imu";
static const std::string HG_STATE_TOPIC = "lowstate";

template <typename T>
class DataBuffer {
 public:
  void SetData(const T &new_data) {
    std::lock_guard<std::mutex> const lock(mutex_);
    data_ = std::make_shared<T>(new_data);
  }

  std::shared_ptr<const T> GetData() {
    std::lock_guard<std::mutex> const lock(mutex_);
    return data_ ? data_ : nullptr;
  }

  void Clear() {
    std::lock_guard<std::mutex> lock(mutex_);
    data_ = nullptr;
  }

 private:
  std::shared_ptr<T> data_;
  std::mutex mutex_;
};

const int G1_NUM_MOTOR = 29;
struct ImuState {
  std::array<float, 3> rpy = {};
  std::array<float, 3> omega = {};
};
struct MotorCommand {
  std::array<float, G1_NUM_MOTOR> q_target = {};
  std::array<float, G1_NUM_MOTOR> dq_target = {};
  std::array<float, G1_NUM_MOTOR> kp = {};
  std::array<float, G1_NUM_MOTOR> kd = {};
  std::array<float, G1_NUM_MOTOR> tau_ff = {};
};
struct MotorState {
  std::array<float, G1_NUM_MOTOR> q = {};
  std::array<float, G1_NUM_MOTOR> dq = {};
};

// Stiffness for all G1 Joints
std::array<float, G1_NUM_MOTOR> Kp{
    60, 60, 60, 100, 40, 40,      // legs
    60, 60, 60, 100, 40, 40,      // legs
    60, 40, 40,                   // waist
    40, 40, 40, 40,  40, 40, 40,  // arms
    40, 40, 40, 40,  40, 40, 40   // arms
};

// Damping for all G1 Joints
std::array<float, G1_NUM_MOTOR> Kd{
    1, 1, 1, 2, 1, 1,     // legs
    1, 1, 1, 2, 1, 1,     // legs
    1, 1, 1,              // waist
    1, 1, 1, 1, 1, 1, 1,  // arms
    1, 1, 1, 1, 1, 1, 1   // arms
};

enum class Mode {
  PR = 0,  // Series Control for Ptich/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};

enum G1JointIndex {
  LEFT_HIP_PITCH = 0,
  LEFT_HIP_ROLL = 1,
  LEFT_HIP_YAW = 2,
  LEFT_KNEE = 3,
  LEFT_ANKLE_PITCH = 4,
  LEFT_ANKLE_B = 4,
  LEFT_ANKLE_ROLL = 5,
  LEFT_ANKLE_A = 5,
  RIGHT_HIP_PITCH = 6,
  RIGHT_HIP_ROLL = 7,
  RIGHT_HIP_YAW = 8,
  RIGHT_KNEE = 9,
  RIGHT_ANKLE_PITCH = 10,
  RIGHT_ANKLE_B = 10,
  RIGHT_ANKLE_ROLL = 11,
  RIGHT_ANKLE_A = 11,
  WAIST_YAW = 12,
  WAIST_ROLL = 13,   // NOTE INVALID for g1 23dof/29dof with waist locked
  WAIST_A = 13,      // NOTE INVALID for g1 23dof/29dof with waist locked
  WAIST_PITCH = 14,  // NOTE INVALID for g1 23dof/29dof with waist locked
  WAIST_B = 14,      // NOTE INVALID for g1 23dof/29dof with waist locked
  LEFT_SHOULDER_PITCH = 15,
  LEFT_SHOULDER_ROLL = 16,
  LEFT_SHOULDER_YAW = 17,
  LEFT_ELBOW = 18,
  LEFT_WRIST_ROLL = 19,
  LEFT_WRIST_PITCH = 20,  // NOTE INVALID for g1 23dof
  LEFT_WRIST_YAW = 21,    // NOTE INVALID for g1 23dof
  RIGHT_SHOULDER_PITCH = 22,
  RIGHT_SHOULDER_ROLL = 23,
  RIGHT_SHOULDER_YAW = 24,
  RIGHT_ELBOW = 25,
  RIGHT_WRIST_ROLL = 26,
  RIGHT_WRIST_PITCH = 27,  // NOTE INVALID for g1 23dof
  RIGHT_WRIST_YAW = 28     // NOTE INVALID for g1 23dof
};

class g1_ankle_swing_sender : public rclcpp::Node {
 public:
  g1_ankle_swing_sender()
      : Node("g1_ankle_swing_sender"),

        mode_machine_(0) {
    //  bind to g1_ankle_swing_sender::LowStateHandler for subscribe "lowstate"
    //  topic
    lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
        HG_STATE_TOPIC, 10,
        [this](const unitree_hg::msg::LowState::SharedPtr message) {
          LowStateHandler(message);
        });
    //  bind to g1_ankle_swing_sender::ImuStateHandler for subscribe
    //  "secondary_imu" topic
    imustate_subscriber_ = this->create_subscription<unitree_hg::msg::IMUState>(
        HG_IMU_TORSO, 10,
        [this](const unitree_hg::msg::IMUState::SharedPtr message) {
          ImuStateHandler(message);
        });

    // the mLowcmdPublisher is set to publish "/lowcmd" topic
    lowcmd_publisher_ =
        this->create_publisher<unitree_hg::msg::LowCmd>(HG_CMD_TOPIC, 10);

    timer1_ = this->create_wall_timer(std::chrono::milliseconds(2),
                                      [this] { Control(); });
    timer2_ = this->create_wall_timer(std::chrono::milliseconds(2),
                                      [this] { LowCommandWriter(); });
  }

 private:
  void Control() {
    MotorCommand motorCommandTmp;
    const std::shared_ptr<const MotorState> ms = motor_state_buffer_.GetData();

    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      motorCommandTmp.tau_ff.at(i) = 0.0;
      motorCommandTmp.q_target.at(i) = 0.0;
      motorCommandTmp.dq_target.at(i) = 0.0;
      motorCommandTmp.kp.at(i) = Kp[i];
      motorCommandTmp.kd.at(i) = Kd[i];
    }

    if (ms) {
      mTime += mControlDt;
      if (mTime < mDuration) {
        // [Stage 1]: set robot to zero posture
        for (int i = 0; i < G1_NUM_MOTOR; ++i) {
          double const ratio =
              unitree::common::clamp(mTime / mDuration, 0.0, 1.0);
          motorCommandTmp.q_target.at(i) = (1.0 - ratio) * ms->q.at(i);
        }
      } else if (mTime < mDuration * 2) {
        // [Stage 2]: swing ankle using PR mode
        mModePr = Mode::PR;
        double const max_P = M_PI * 30.0 / 180.0;
        double const max_R = M_PI * 10.0 / 180.0;
        double const t = mTime - mDuration;
        double const L_P_des = max_P * std::sin(2.0 * M_PI * t);
        double const L_R_des = max_R * std::sin(2.0 * M_PI * t);
        double const R_P_des = max_P * std::sin(2.0 * M_PI * t);
        double const R_R_des = -max_R * std::sin(2.0 * M_PI * t);

        motorCommandTmp.q_target.at(LEFT_ANKLE_PITCH) = L_P_des;
        motorCommandTmp.q_target.at(LEFT_ANKLE_ROLL) = L_R_des;
        motorCommandTmp.q_target.at(RIGHT_ANKLE_PITCH) = R_P_des;
        motorCommandTmp.q_target.at(RIGHT_ANKLE_ROLL) = R_R_des;
      } else {
        // [Stage 3]: swing ankle using AB mode
        mModePr = Mode::AB;
        double const max_A = M_PI * 30.0 / 180.0;
        double const max_B = M_PI * 10.0 / 180.0;
        double const t = mTime - mDuration * 2;
        double const L_A_des = +max_A * std::sin(M_PI * t);
        double const L_B_des = +max_B * std::sin(M_PI * t + M_PI);
        double const R_A_des = -max_A * std::sin(M_PI * t);
        double const R_B_des = -max_B * std::sin(M_PI * t + M_PI);

        motorCommandTmp.q_target.at(LEFT_ANKLE_A) = L_A_des;
        motorCommandTmp.q_target.at(LEFT_ANKLE_B) = L_B_des;
        motorCommandTmp.q_target.at(RIGHT_ANKLE_A) = R_A_des;
        motorCommandTmp.q_target.at(RIGHT_ANKLE_B) = R_B_des;
      }

      motor_command_buffer_.SetData(motorCommandTmp);
    }
  }

  void LowCommandWriter() {
    unitree_hg::msg::LowCmd lowCommand;
    lowCommand.mode_pr = static_cast<uint8_t>(mModePr);
    lowCommand.mode_machine = mode_machine_;

    const std::shared_ptr<const MotorCommand> mc =
        motor_command_buffer_.GetData();
    if (mc) {
      for (size_t i = 0; i < G1_NUM_MOTOR; i++) {
        lowCommand.motor_cmd.at(i).mode = 1;  // 1:Enable, 0:Disable
        lowCommand.motor_cmd.at(i).tau = mc->tau_ff.at(i);
        lowCommand.motor_cmd.at(i).q = mc->q_target.at(i);
        lowCommand.motor_cmd.at(i).dq = mc->dq_target.at(i);
        lowCommand.motor_cmd.at(i).kp = mc->kp.at(i);
        lowCommand.motor_cmd.at(i).kd = mc->kd.at(i);
      }

      get_crc(lowCommand);
      lowcmd_publisher_->publish(lowCommand);
    }
  }

  void LowStateHandler(const unitree_hg::msg::LowState::SharedPtr &message) {
    // get motor state
    MotorState msTmp;
    for (int i = 0; i < G1_NUM_MOTOR; ++i) {
      msTmp.q.at(i) = message->motor_state[i].q;
      msTmp.dq.at(i) = message->motor_state[i].dq;
      if ((message->motor_state[i].motorstate != 0u) && i <= RIGHT_ANKLE_ROLL) {
        RCLCPP_INFO(this->get_logger(), "[ERROR] motor %d with code %d", i,
                    message->motor_state[i].motorstate);
      }
    }
    motor_state_buffer_.SetData(msTmp);

    // get imu state
    ImuState imuTmp;
    imuTmp.omega = message->imu_state.gyroscope;
    imuTmp.rpy = message->imu_state.rpy;
    imu_state_buffer_.SetData(imuTmp);

    // update gamepad
    Gamepad gamepad;
    REMOTE_DATA_RX rx;
    memcpy(rx.buff, message->wireless_remote.data(), 40);
    gamepad.update(rx.RF_RX);

    // update mode machine
    if (mode_machine_ != message->mode_machine) {
      if (mode_machine_ == 0) {
        RCLCPP_INFO(this->get_logger(), "G1 type: %d",
                    unsigned(message->mode_machine));
      }
      mode_machine_ = message->mode_machine;
    }

    // report robot status every second
    if (++mCounter % 500 == 0) {
      mCounter = 0;
      // IMU
      auto &rpy = message->imu_state.rpy;
      RCLCPP_INFO(this->get_logger(), "IMU.pelvis.rpy: %.2f %.2f %.2f\n",
                  rpy[0], rpy[1], rpy[2]);

      // RC
      RCLCPP_INFO(this->get_logger(), "gamepad.A.pressed: %d\n",
                  static_cast<int>(gamepad.A.pressed));
      RCLCPP_INFO(this->get_logger(), "gamepad.B.pressed: %d\n",
                  static_cast<int>(gamepad.B.pressed));
      RCLCPP_INFO(this->get_logger(), "gamepad.X.pressed: %d\n",
                  static_cast<int>(gamepad.X.pressed));
      RCLCPP_INFO(this->get_logger(), "gamepad.Y.pressed: %d\n",
                  static_cast<int>(gamepad.Y.pressed));

      // Motor
      auto &ms = message->motor_state;
      RCLCPP_INFO(this->get_logger(), "All %d Motors:", G1_NUM_MOTOR);
      std::ostringstream oss;
      oss.str("");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        oss << static_cast<int32_t>(ms[i].mode) << " ";
      }
      RCLCPP_INFO(this->get_logger(), "mode: %s", oss.str().c_str());
      oss.str("");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        oss << std::fixed << std::setprecision(2) << ms[i].q << " ";
      }
      RCLCPP_INFO(this->get_logger(), "pos: %s", oss.str().c_str());
      oss.str("");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        oss << std::fixed << std::setprecision(2) << ms[i].dq << " ";
      }
      RCLCPP_INFO(this->get_logger(), "vel: %s", oss.str().c_str());
      oss.str("");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        oss << std::fixed << std::setprecision(2) << ms[i].tau_est << " ";
      }
      RCLCPP_INFO(this->get_logger(), "tau_est: %s", oss.str().c_str());
      oss.str("");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        oss << ms[i].temperature[0] << "," << ms[i].temperature[1] << " ";
      }
      RCLCPP_INFO(this->get_logger(), "temperature: %s", oss.str().c_str());
      oss.str("");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        oss << std::fixed << std::setprecision(2) << ms[i].vol << " ";
      }
      RCLCPP_INFO(this->get_logger(), "vol: %s", oss.str().c_str());
      oss.str("");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        oss << ms[i].sensor[0] << "," << ms[i].sensor[1] << " ";
      }
      RCLCPP_INFO(this->get_logger(), "sensor: %s", oss.str().c_str());
      oss.str("");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        oss << ms[i].motorstate << " ";
      }
      RCLCPP_INFO(this->get_logger(), "motorstate: %s", oss.str().c_str());
      oss.str("");
      for (int i = 0; i < G1_NUM_MOTOR; ++i) {
        oss << ms[i].reserve[0] << "," << ms[i].reserve[1] << ","
            << ms[i].reserve[2] << ms[i].reserve[3] << " ";
      }
      RCLCPP_INFO(this->get_logger(), "reserve: %s", oss.str().c_str());
    }
  }

  void ImuStateHandler(const unitree_hg::msg::IMUState::SharedPtr &message) {
    auto &rpy = message->rpy;
    if (mCounter % 500 == 0) {
      RCLCPP_INFO(this->get_logger(), "IMU.torso.rpy: %.2f %.2f %.2f", rpy[0],
                  rpy[1], rpy[2]);
    }
  }

  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr
      lowcmd_publisher_;  // ROS2 Publisher
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr
      lowstate_subscriber_;  // ROS2 Subscriber
  rclcpp::Subscription<unitree_hg::msg::IMUState>::SharedPtr
      imustate_subscriber_;  // ROS2 Subscriber
  rclcpp::TimerBase::SharedPtr timer1_;
  rclcpp::TimerBase::SharedPtr timer2_;

  double mTime{0.0};
  double mControlDt{0.002};  // [2ms]
  double mDuration{3.0};     // [3 s]
  int32_t mCounter{0};
  Mode mModePr{Mode::PR};
  std::atomic<uint8_t> mode_machine_;

  DataBuffer<MotorState> motor_state_buffer_;
  DataBuffer<MotorCommand> motor_command_buffer_;
  DataBuffer<ImuState> imu_state_buffer_;
};

// try to shutdown motion control-related service
void ShutdownMotionCtrl() {
  auto client = std::make_shared<unitree::ros2::MotionSwitchClient>();
  std::string form;
  std::string name;
  while (client->CheckMode(form, name), !name.empty()) {
    if (client->ReleaseMode() != 0) {
      std::cout << "Failed to switch to Release Mode" << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
  }
  std::cout << "Shutdown motion control success" << std::endl;
}

int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);  // Initialize rclcpp
    ShutdownMotionCtrl();

    auto node = std::make_shared<
        g1_ankle_swing_sender>();  // Create a ROS2 node and make share with
                                   // g1_ankle_swing_sender class
    rclcpp::spin(node);            // Run ROS2 node
    rclcpp::shutdown();            // Exit
  } catch (const rclcpp::exceptions::RCLError &e) {
    std::cerr << "RCLError caught: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}