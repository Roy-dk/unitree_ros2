/**
 * This example demonstrates how to use ROS2 to control arm commands of unitree
 *g1 robot
 **/
#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/motor_cmd.hpp"
#include "utils/utils.hpp"

using std::placeholders::_1;

constexpr float K_PI = 3.141592654;
constexpr float K_PI_2 = 1.57079632;

enum JointIndex {
  // Left leg
  K_LEFT_HIP_PITCH,
  K_LEFT_HIP_ROLL,
  K_LEFT_HIP_YAW,
  K_LEFT_KNEE,
  K_LEFT_ANKLE,
  K_LEFT_ANKLE_ROLL,

  // Right leg
  K_RIGHT_HIP_PITCH,
  K_RIGHT_HIP_ROLL,
  K_RIGHT_HIP_YAW,
  K_RIGHT_KNEE,
  K_RIGHT_ANKLE,
  K_RIGHT_ANKLE_ROLL,

  K_WAIST_YAW,
  K_WAIST_ROLL,
  K_WAIST_PITCH,

  // Left arm
  K_LEFT_SHOULDER_PITCH,
  K_LEFT_SHOULDER_ROLL,
  K_LEFT_SHOULDER_YAW,
  K_LEFT_ELBOW,
  K_LEFT_WIST_ROLL,
  K_LEFT_WIST_PITCH,
  K_LEFT_WIST_YAW,
  // Right arm
  K_RIGHT_SHOULDER_PITCH,
  K_RIGHT_SHOULDER_ROLL,
  K_RIGHT_SHOULDER_YAW,
  K_RIGHT_ELBOW,
  K_RIGHT_WIST_ROLL,
  K_RIGHT_WIST_PITCH,
  K_RIGHT_WIST_YAW,

  K_NOT_USED_JOINT,
  K_NOT_USED_JOINT1,
  K_NOT_USED_JOINT2,
  K_NOT_USED_JOINT3,
  K_NOT_USED_JOINT4,
  K_NOT_USED_JOINT5
};

struct LowCmdParam {
  float period = 5.F;
  float init_time = 2.0F;
  float control_dt = 0.02F;
  float weight = 0.F;
  float weight_rate = 0.2F;
  float kp = 60.F;
  float kd = 1.5F;
  float dq = 0.F;
  float tau_ff = 0.F;
  float max_joint_velocity = 0.5F;
  float delta_weight = weight_rate * control_dt;
  float max_joint_delta = max_joint_velocity * control_dt;
  int32_t sleep_time = static_cast<int>(control_dt / 0.001F);
  int32_t num_time_steps = static_cast<int>(period / control_dt);
  std::array<float, 17> init_pos = {0, 0, 0, 0, 0, 0, 0, 0, 0,
                                    0, 0, 0, 0, 0, 0, 0, 0};
  std::array<float, 17> target_pos = {0.F, K_PI_2, 0.F,     K_PI_2, 0.F,    0.F,
                                      0.F, 0.F,    -K_PI_2, 0.F,    K_PI_2, 0.F,
                                      0.F, 0.F,    0,       0,      0};
  std::array<JointIndex, 17> arm_joints = {
      JointIndex::K_LEFT_SHOULDER_PITCH, JointIndex::K_LEFT_SHOULDER_ROLL,
      JointIndex::K_LEFT_SHOULDER_YAW,   JointIndex::K_LEFT_ELBOW,
      JointIndex::K_LEFT_WIST_ROLL,      JointIndex::K_LEFT_WIST_PITCH,
      JointIndex::K_LEFT_WIST_YAW,       JointIndex::K_RIGHT_SHOULDER_PITCH,
      JointIndex::K_RIGHT_SHOULDER_ROLL, JointIndex::K_RIGHT_SHOULDER_YAW,
      JointIndex::K_RIGHT_ELBOW,         JointIndex::K_RIGHT_WIST_ROLL,
      JointIndex::K_RIGHT_WIST_PITCH,    JointIndex::K_RIGHT_WIST_YAW,
      JointIndex::K_WAIST_YAW,           JointIndex::K_WAIST_ROLL,
      JointIndex::K_WAIST_PITCH};
  std::array<float, 17> current_jpos_des{};
};

// Create a g1_arm7_control_sender class for low state receive
class g1_arm7_control_sender : public rclcpp::Node {
 public:
  g1_arm7_control_sender() : Node("g1_arm7_control_sender") {
    const auto *topic_name = "lowstate";

    //  bind to g1_arm7_control_sender::LowStateHandler for subscribe "lowstate"
    //  topic
    lowstate_subscriber_ = this->create_subscription<unitree_hg::msg::LowState>(
        topic_name, 10,
        [this](const unitree_hg::msg::LowState::SharedPtr message) {
          LowStateHandler(message);
        });

    // the mLowcmdPublisher is set to subscribe "/arm_sdk" topic
    lowcmd_publisher_ =
        this->create_publisher<unitree_hg::msg::LowCmd>("/arm_sdk", 10);
  }

 private:
  void Control() {
    InitArms();
    LiftArmsUp();
    PutArmsDown();
    StopControl();
    timer_->cancel();
  }

  void InitArms() {
    // get current joint position
    std::array<float, 17> current_jpos{};
    std::ostringstream oss;
    for (size_t i = 0; i < low_cmd_param_.arm_joints.size(); ++i) {
      current_jpos.at(i) =
          state_msg_->motor_state.at(low_cmd_param_.arm_joints.at(i)).q;
      oss << current_jpos.at(i) << " ";
    }
    RCLCPP_INFO(this->get_logger(), "Current joint position: %s",
                oss.str().c_str());

    // set init position
    RCLCPP_INFO(this->get_logger(), "Initailizing arms ...");

    int const init_time_steps =
        static_cast<int>(low_cmd_param_.init_time / low_cmd_param_.control_dt);
    for (int i = 0; i < init_time_steps; ++i) {
      // increase weight
      low_cmd_param_.weight = 1.0;
      cmd_.motor_cmd.at(JointIndex::K_NOT_USED_JOINT)
          .set__q(low_cmd_param_.weight);
      float const phase = 1.0 * i / init_time_steps;
      RCLCPP_INFO(this->get_logger(), "Phase: %f", phase);

      // set control joints
      for (size_t j = 0; j < low_cmd_param_.init_pos.size(); ++j) {
        cmd_.motor_cmd.at(low_cmd_param_.arm_joints.at(j))
            .set__q(low_cmd_param_.init_pos.at(j) * phase +
                    current_jpos.at(j) * (1 - phase));
        cmd_.motor_cmd.at(low_cmd_param_.arm_joints.at(j))
            .set__dq(low_cmd_param_.dq);
        cmd_.motor_cmd.at(low_cmd_param_.arm_joints.at(j))
            .set__kp(low_cmd_param_.kp);
        cmd_.motor_cmd.at(low_cmd_param_.arm_joints.at(j))
            .set__kd(low_cmd_param_.kd);
        cmd_.motor_cmd.at(low_cmd_param_.arm_joints.at(j))
            .set__tau(low_cmd_param_.tau_ff);
      }
      // Publish lowcmd message
      lowcmd_publisher_->publish(cmd_);

      // sleep
      std::this_thread::sleep_for(
          std::chrono::milliseconds(low_cmd_param_.sleep_time));
    }
    RCLCPP_INFO(this->get_logger(), "Init Arms Done!");
  }

  void LiftArmsUp() {
    RCLCPP_INFO(this->get_logger(), "Start lift arms up!");
    for (int i = 0; i < low_cmd_param_.num_time_steps; ++i) {
      // update jpos des
      for (size_t j = 0; j < low_cmd_param_.init_pos.size(); ++j) {
        low_cmd_param_.current_jpos_des.at(j) += unitree::common::clamp(
            low_cmd_param_.target_pos.at(j) -
                low_cmd_param_.current_jpos_des.at(j),
            -low_cmd_param_.max_joint_delta, low_cmd_param_.max_joint_delta);
      }

      // set control joints
      for (size_t j = 0; j < low_cmd_param_.init_pos.size(); ++j) {
        cmd_.motor_cmd.at(low_cmd_param_.arm_joints.at(j))
            .set__q(low_cmd_param_.current_jpos_des.at(j));
        cmd_.motor_cmd.at(low_cmd_param_.arm_joints.at(j))
            .set__dq(low_cmd_param_.dq);
        cmd_.motor_cmd.at(low_cmd_param_.arm_joints.at(j))
            .set__kp(low_cmd_param_.kp);
        cmd_.motor_cmd.at(low_cmd_param_.arm_joints.at(j))
            .set__kd(low_cmd_param_.kd);
        cmd_.motor_cmd.at(low_cmd_param_.arm_joints.at(j))
            .set__tau(low_cmd_param_.tau_ff);
      }

      // send ros msg
      lowcmd_publisher_->publish(cmd_);

      // sleep
      std::this_thread::sleep_for(
          std::chrono::milliseconds(low_cmd_param_.sleep_time));
    }
  }

  void PutArmsDown() {
    RCLCPP_INFO(this->get_logger(), "Start lift arms down!");
    for (int32_t i = 0; i < low_cmd_param_.num_time_steps; ++i) {
      // update jpos des
      for (size_t j = 0; j < low_cmd_param_.init_pos.size(); ++j) {
        low_cmd_param_.current_jpos_des.at(j) += unitree::common::clamp(
            low_cmd_param_.init_pos.at(j) -
                low_cmd_param_.current_jpos_des.at(j),
            -low_cmd_param_.max_joint_delta, low_cmd_param_.max_joint_delta);
      }

      // set control joints
      for (size_t j = 0; j < low_cmd_param_.init_pos.size(); ++j) {
        cmd_.motor_cmd.at(low_cmd_param_.arm_joints.at(j))
            .set__q(low_cmd_param_.current_jpos_des.at(j));
        cmd_.motor_cmd.at(low_cmd_param_.arm_joints.at(j))
            .set__dq(low_cmd_param_.dq);
        cmd_.motor_cmd.at(low_cmd_param_.arm_joints.at(j))
            .set__kp(low_cmd_param_.kp);
        cmd_.motor_cmd.at(low_cmd_param_.arm_joints.at(j))
            .set__kd(low_cmd_param_.kd);
        cmd_.motor_cmd.at(low_cmd_param_.arm_joints.at(j))
            .set__tau(low_cmd_param_.tau_ff);
      }

      // send ros msg
      lowcmd_publisher_->publish(cmd_);

      // sleep
      std::this_thread::sleep_for(
          std::chrono::milliseconds(low_cmd_param_.sleep_time));
    }
  }

  void StopControl() {
    RCLCPP_INFO(this->get_logger(), "Stoping arm ctrl ...");
    float const stop_time = 2.0F;
    int const stop_time_steps =
        static_cast<int>(stop_time / low_cmd_param_.control_dt);

    for (int i = 0; i < stop_time_steps; ++i) {
      // increase weight
      low_cmd_param_.weight -= low_cmd_param_.delta_weight;
      low_cmd_param_.weight =
          unitree::common::clamp(low_cmd_param_.weight, 0.F, 1.F);

      // set weight
      cmd_.motor_cmd.at(JointIndex::K_NOT_USED_JOINT)
          .set__q(low_cmd_param_.weight);

      // send ros msg
      lowcmd_publisher_->publish(cmd_);

      // sleep
      std::this_thread::sleep_for(
          std::chrono::milliseconds(low_cmd_param_.sleep_time));
    }
  }

  void LowStateHandler(const unitree_hg::msg::LowState::SharedPtr &message) {
    if (state_msg_ == nullptr) {
      std::lock_guard<std::mutex> const mtx(mtx_);
      if (state_msg_ == nullptr) {
        state_msg_ = std::make_shared<unitree_hg::msg::LowState>();

        state_msg_->set__version(message->version);
        state_msg_->set__mode_pr(message->mode_pr);
        state_msg_->set__mode_machine(message->mode_machine);
        state_msg_->set__tick(message->tick);
        state_msg_->set__imu_state(message->imu_state);
        state_msg_->set__motor_state(message->motor_state);
        state_msg_->set__wireless_remote(message->wireless_remote);
        state_msg_->set__reserve(message->reserve);
        state_msg_->set__crc(message->crc);

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         [this] { Control(); });
      }
    }
  }

  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr
      lowcmd_publisher_;  // ROS2 Publisher
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr
      lowstate_subscriber_;  // ROS2 Subscriber
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mtx_;
  unitree_hg::msg::LowState::SharedPtr state_msg_;
  unitree_hg::msg::LowCmd cmd_;
  LowCmdParam low_cmd_param_;
};

int main(int argc, char **argv) {
  try {
    rclcpp::init(argc, argv);  // Initialize rclcpp
    auto node = std::make_shared<
        g1_arm7_control_sender>();  // Create a ROS2 node and make share with
                                    // g1_arm7_control_sender class
    rclcpp::spin(node);             // Run ROS2 node
    rclcpp::shutdown();             // Exit
  } catch (const rclcpp::exceptions::RCLError &e) {
    std::cerr << "RCLError caught: " << e.what() << std::endl;
    return 1;
  }
  return 0;
}