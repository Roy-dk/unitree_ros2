/**
 * This example demonstrates how to use ROS2 to control hands of unitree g1
 *robot
 **/
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/hand_cmd.hpp"
#include "unitree_hg/msg/hand_state.hpp"
#include "utils/utils.hpp"

int32_t MOTOR_MAX = 7;
int32_t SENSOR_MAX = 9;
uint8_t hand_id = 0;
std::string handcmd_topic = "dex3/left/cmd";
std::string handstate_topic = "lf/dex3/left/state";

// set URDF Limits
const float maxLimits_left[7] = {1.05, 1.05, 1.75, 0,
                                 0,    0,    0};  // set max motor value
const float minLimits_left[7] = {-1.05, -0.724, 0, -1.57, -1.75, -1.57, -1.75};
const float maxLimits_right[7] = {1.05, 0.742, 0, 1.57, 1.75, 1.57, 1.75};
const float minLimits_right[7] = {-1.05, -1.05, -1.75, 0, 0, 0, 0};

enum State { INIT, ROTATE, GRIP, STOP, PRINT };

using RIS_Mode_t = struct {
  uint8_t id : 4;
  uint8_t status : 3;
  uint8_t timeout : 1;
};

class G1Dex3Sender : public rclcpp::Node {
 public:
  G1Dex3Sender() : Node("G1Dex3Sender"), current_state_(INIT) {
    // The suber  callback function is bind to G1Dex3Sender::topic_callback
    handstate_subscriber_ =
        this->create_subscription<unitree_hg::msg::HandState>(
            handstate_topic, 10,
            [this](const unitree_hg::msg::HandState::SharedPtr message) {
              HandStateHandler(message);
            });

    // the handcmd_publisher_ is set to subscribe "lf/dex3/[left or
    // right]/state" topic
    handcmd_publisher_ =
        this->create_publisher<unitree_hg::msg::HandCmd>(handcmd_topic, 10);

    // The timer is set to read user input
    timer_ = this->create_wall_timer(std::chrono::milliseconds(100),
                                     [this] { UserInputHandle(); });

    thread_ = std::thread([this] { HandControlHandler(); });
  }
  ~G1Dex3Sender() override {
    running_ = false;
    thread_.join();
  }

 private:
  void HandStateHandler(const unitree_hg::msg::HandState::SharedPtr &message) {
    std::lock_guard<std::mutex> const lck(state_mtx_);
    if (!handState_) {
      handState_ = std::make_shared<unitree_hg::msg::HandState>(*message.get());
    } else {
      *handState_.get() = *message.get();
    }
  }

  void HandControlHandler() {
    running_ = true;
    State lastState = INIT;
    while (running_) {
      State const state = current_state_.load();
      if (state != lastState) {
        RCLCPP_INFO(this->get_logger(), "--- Current State: %s ---",
                    stateToString(state));
        RCLCPP_INFO(this->get_logger(), "Commands:");
        RCLCPP_INFO(this->get_logger(), "  r - Rotate");
        RCLCPP_INFO(this->get_logger(), "  g - Grip");
        RCLCPP_INFO(this->get_logger(), "  p - Print_state");
        RCLCPP_INFO(this->get_logger(), "  q - Quit");
        RCLCPP_INFO(this->get_logger(), "  s - Stop");
        lastState = state;
      }

      switch (state) {
        case INIT:
          RCLCPP_INFO(this->get_logger(), "Initializing...");
          current_state_ = ROTATE;
          break;
        case ROTATE:
          rotateMotors(hand_id == 0);
          break;
        case GRIP:
          gripHand(hand_id == 0);
          break;
        case STOP:
          stopMotors();
          break;
        case PRINT:
          printState(hand_id == 0);
          break;
        default:
          RCLCPP_ERROR(this->get_logger(), "Invalid state!");
          break;
      }
    }
  }

  void UserInputHandle() {
    char const ch = getNonBlockingInput();
    if (ch == 'q') {
      RCLCPP_INFO(this->get_logger(), "Exiting...");
      current_state_ = STOP;
      timer_->cancel();
      running_ = false;
    } else if (ch == 'r') {
      current_state_ = ROTATE;
    } else if (ch == 'g') {
      current_state_ = GRIP;
    } else if (ch == 'p') {
      current_state_ = PRINT;
    } else if (ch == 's') {
      current_state_ = STOP;
    }
  }

  static char getNonBlockingInput() {
    struct termios oldt;
    struct termios newt;
    char ch = 0;
    int oldf = 0;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    return ch;
  }

  static const char *stateToString(State state) {
    switch (state) {
      case INIT:
        return "INIT";
      case ROTATE:
        return "ROTATE";
      case GRIP:
        return "GRIP";
      case STOP:
        return "STOP";
      case PRINT:
        return "PRINT";
      default:
        return "UNKNOWN";
    }
  }

  // this method can send kp and kd to motors
  void rotateMotors(bool is_left_hand) {
    static int _count = 1;
    static int dir = 1;
    const float *maxLimits = is_left_hand ? maxLimits_left : maxLimits_right;
    const float *minLimits = is_left_hand ? minLimits_left : minLimits_right;

    unitree_hg::msg::HandCmd msg;
    msg.motor_cmd.resize(MOTOR_MAX);
    for (int i = 0; i < MOTOR_MAX; i++) {
      RIS_Mode_t ris_mode;
      ris_mode.id = i;
      ris_mode.status = 0x01;

      uint8_t mode = 0;
      mode |= (ris_mode.id & 0x0F);
      mode |= (ris_mode.status & 0x07) << 4;
      mode |= (ris_mode.timeout & 0x01) << 7;
      msg.motor_cmd[i].set__mode(mode);
      msg.motor_cmd[i].set__tau(0);
      msg.motor_cmd[i].set__kp(0.5);
      msg.motor_cmd[i].set__kd(0.1);

      float const range = maxLimits[i] - minLimits[i];
      float const mid = (maxLimits[i] + minLimits[i]) / 2.0;
      float const amplitude = range / 2.0;
      float const q = mid + amplitude * sin(_count / 20000.0 * M_PI);

      msg.motor_cmd[i].set__q(q);
    }

    handcmd_publisher_->publish(msg);
    _count += dir;

    if (_count >= 10000) {
      dir = -1;
    }
    if (_count <= -10000) {
      dir = 1;
    }

    usleep(100);
  }

  // this method can send static position to motors
  void gripHand(bool is_left_hand) {
    const float *maxLimits = is_left_hand ? maxLimits_left : maxLimits_right;
    const float *minLimits = is_left_hand ? minLimits_left : minLimits_right;

    unitree_hg::msg::HandCmd msg;
    msg.motor_cmd.resize(MOTOR_MAX);
    for (int32_t i = 0; i < MOTOR_MAX; i++) {
      RIS_Mode_t ris_mode;
      ris_mode.id = i;
      ris_mode.status = 0x01;

      uint8_t mode = 0;
      mode |= (ris_mode.id & 0x0F);
      mode |= (ris_mode.status & 0x07) << 4;
      mode |= (ris_mode.timeout & 0x01) << 7;
      msg.motor_cmd[i].set__mode(mode);
      msg.motor_cmd[i].set__tau(0);

      float const mid = (maxLimits[i] + minLimits[i]) / 2.0;

      msg.motor_cmd[i].set__q(mid);
      msg.motor_cmd[i].set__dq(0);
      msg.motor_cmd[i].set__kp(1.5);
      msg.motor_cmd[i].set__kd(0.1);
    }

    handcmd_publisher_->publish(msg);
    usleep(1000000);
  }

  // this method can subscribe dds and show the position for now
  void printState(bool is_left_hand) {
    unitree_hg::msg::HandState handState;
    {
      std::lock_guard<std::mutex> const lck(state_mtx_);
      if (!handState_) {
        RCLCPP_ERROR(this->get_logger(), "hand state is nullptr");
        return;
      }
      handState = *handState_.get();
    }
    Eigen::Matrix<float, 7, 1> q;

    const float *maxLimits = is_left_hand ? maxLimits_left : maxLimits_right;
    const float *minLimits = is_left_hand ? minLimits_left : minLimits_right;
    for (int i = 0; i < 7; i++) {
      q(i) = handState.motor_state[i].q;

      q(i) = (q(i) - minLimits[i]) / (maxLimits[i] - minLimits[i]);
      q(i) = unitree::common::clamp(q(i), 0.0F, 1.0F);
    }
    RCLCPP_INFO(this->get_logger(), "\033[2J\033[H");
    RCLCPP_INFO(this->get_logger(), "-- Hand State --");
    RCLCPP_INFO(this->get_logger(), "--- Current State: Test ---");
    RCLCPP_INFO(this->get_logger(), "Commands:");
    RCLCPP_INFO(this->get_logger(), "  r - Rotate");
    RCLCPP_INFO(this->get_logger(), "  g - Grip");
    RCLCPP_INFO(this->get_logger(), "  t - Test");
    RCLCPP_INFO(this->get_logger(), "  q - Quit");
    if (is_left_hand) {
      std::cout << " L: " << q.transpose() << std::endl;
    } else {
      std::cout << " R: " << q.transpose() << std::endl;
    }
    usleep(0.1 * 1e6);
  }

  // this method can send dynamic position to motors
  void stopMotors() {
    unitree_hg::msg::HandCmd msg;
    msg.motor_cmd.resize(MOTOR_MAX);
    for (int i = 0; i < MOTOR_MAX; i++) {
      RIS_Mode_t ris_mode;
      ris_mode.id = i;
      ris_mode.status = 0x01;
      ris_mode.timeout = 0x01;

      uint8_t mode = 0;
      mode |= (ris_mode.id & 0x0F);
      mode |= (ris_mode.status & 0x07) << 4;
      mode |= (ris_mode.timeout & 0x01) << 7;
      msg.motor_cmd[i].set__mode(mode);
      msg.motor_cmd[i].set__tau(0);
      msg.motor_cmd[i].set__dq(0);
      msg.motor_cmd[i].set__kp(0);
      msg.motor_cmd[i].set__kd(0);
      msg.motor_cmd[i].set__q(0);
    }
    handcmd_publisher_->publish(msg);
    usleep(1000000);
  }

  rclcpp::TimerBase::SharedPtr timer_;  // ROS2 timer
  rclcpp::Publisher<unitree_hg::msg::HandCmd>::SharedPtr
      handcmd_publisher_;  // ROS2 Publisher
  rclcpp::Subscription<unitree_hg::msg::HandState>::SharedPtr
      handstate_subscriber_;          // ROS2 Subscriber
  std::thread thread_;                // Loop publish thread
  std::atomic<State> current_state_;  // Current hand control state
  bool running_{};                    // thread_ running flag
  unitree_hg::msg::HandState::SharedPtr handState_;  // HandState Message
  std::mutex state_mtx_;                             // HandState mutex
};

int main(int argc, char **argv) {
  // select which hand to control
  std::cout << " --- Unitree Robotics --- \n";
  std::cout << "     Dex3 Hand Example      \n\n";
  std::string input;
  std::cout << "Please input the hand id (L for left hand, R for right hand): ";
  std::cin >> input;

  if (input == "L") {
    hand_id = 0;
    handcmd_topic = "dex3/left/cmd";
    handstate_topic = "lf/dex3/left/state";
  } else if (input == "R") {
    hand_id = 1;
    handcmd_topic = "dex3/right/cmd";
    handstate_topic = "lf/dex3/right/state";
  } else {
    std::cout << "Invalid hand id. Please input 'L' or 'R'." << std::endl;
    return -1;
  }

  // start hand control
  rclcpp::init(argc, argv);  // Initialize rclcpp
  auto node =
      std::make_shared<G1Dex3Sender>();  // Create a ROS2 node and make share
                                         // with low_level_cmd_sender class
  rclcpp::spin(node);                    // Run ROS2 node
  rclcpp::shutdown();                    // Exit
  return 0;
}