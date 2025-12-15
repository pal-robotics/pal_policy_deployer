#include "pal_policy_preprocessor/preprocessor_tiago_ik_node.hpp"

#include <algorithm>
#include <chrono>

using std::placeholders::_1;

IKObsPublisher::IKObsPublisher()
: rclcpp::Node("onnx_input_publisher"), obs_(59, 0.0)
{
  // === Initial left command pose (xyz, qw, qx, qy, qz) ===
  obs_[45] = 0.27634;       // left x
  obs_[46] = 0.20833;       // left y
  obs_[47] = 0.54071;       // left z

  obs_[48] = 0.078844;      // left qw
  obs_[49] = 0.21089;       // left qx
  obs_[50] = 0.00096864;    // left qy
  obs_[51] = 0.97432;       // left qz

  // === Initial right command pose (xyz, qw, qx, qy, qz) ===
  obs_[52] = 0.27627;       // right x
  obs_[53] = -0.20836;      // right y
  obs_[54] = 0.54072;       // right z

  obs_[55] = -0.078344;     // right qw
  obs_[56] = 0.21066;       // right qx
  obs_[57] = 0.0041078;     // right qy
  obs_[58] = 0.97441;       // right qz

  // Subscriptions
  sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 1, std::bind(&IKObsPublisher::jsCB, this, _1));

  sub_left_pos_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/left_target_position", 1, std::bind(&IKObsPublisher::leftPosCB, this, _1));
  sub_left_quat_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
    "/left_target_orientation", 1, std::bind(&IKObsPublisher::leftQuatCB, this, _1));

  sub_right_pos_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/right_target_position", 1, std::bind(&IKObsPublisher::rightPosCB, this, _1));
  sub_right_quat_ = this->create_subscription<geometry_msgs::msg::Quaternion>(
    "/right_target_orientation", 1, std::bind(&IKObsPublisher::rightQuatCB, this, _1));

  sub_last_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/ik_policy/last_action", 1, std::bind(&IKObsPublisher::lastCB, this, _1));

  // Publisher
  pub_obs_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ik_policy/obs", 1);

  // Timer at 50 Hz (20 ms)
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&IKObsPublisher::onTimer, this));
}

// JointState callback: fill positions and velocities
// New layout:
//   obs[ 0..14]  = positions    (left, right, torso)
//   obs[15..29]  = velocities   (left, right, torso)
void IKObsPublisher::jsCB(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // Safety check (optional, but helps avoid out-of-range)
  if (msg->position.size() <= 18 || msg->velocity.size() <= 18) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "JointState message too small: pos size %zu, vel size %zu",
      msg->position.size(), msg->velocity.size());
    return;
  }


  // === Positions: torso, left, right ===
  obs_[0] = msg->position[18];          // torso

  // left arm -> obs_[1..7]
  for (int i = 0; i < 7; ++i) {
    obs_[1 + i] = msg->position[i];
  }

  // right arm -> obs_[8..14]
  for (int i = 0; i < 7; ++i) {
    obs_[8 + i] = msg->position[7 + i];
  }

  // === Velocities: torso, left, right ===
  obs_[15] = msg->velocity[18];         // torso

  // left arm -> obs_[16..22]
  for (int i = 0; i < 7; ++i) {
    obs_[16 + i] = msg->velocity[i];
  }

  // right arm -> obs_[23..29]
  for (int i = 0; i < 7; ++i) {
    obs_[23 + i] = msg->velocity[7 + i];
  }
}

// Target pose callbacks: commands (left/right)
// Layout:
//   obs[45..51] = left  command (xyz, qw, qx, qy, qz)
//   obs[52..58] = right command (xyz, qw, qx, qy, qz)
void IKObsPublisher::leftPosCB(const geometry_msgs::msg::Point::SharedPtr msg)
{
  // Left command position -> obs_[45..47]
  obs_[45] = msg->x;
  obs_[46] = msg->y;
  obs_[47] = msg->z;
}

void IKObsPublisher::leftQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg)
{
  // ROS gives (x,y,z,w), policy expects (w,x,y,z)
  // Left command orientation -> obs_[48..51]
  obs_[48] = msg->w;
  obs_[49] = msg->x;
  obs_[50] = msg->y;
  obs_[51] = msg->z;
}

void IKObsPublisher::rightPosCB(const geometry_msgs::msg::Point::SharedPtr msg)
{
  // Right command position -> obs_[52..54]
  obs_[52] = msg->x;
  obs_[53] = msg->y;
  obs_[54] = msg->z;
}

void IKObsPublisher::rightQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg)
{
  // ROS gives (x,y,z,w), policy expects (w,x,y,z)
  // Right command orientation -> obs_[55..58]
  obs_[55] = msg->w;
  obs_[56] = msg->x;
  obs_[57] = msg->y;
  obs_[58] = msg->z;
}

void IKObsPublisher::lastCB(const std_msgs::msg::Float64MultiArray::SharedPtr a)
{
  // Actions for 15 joints -> obs_[30..44]
  // Layout in /ik_policy/last_action:
  //   [0..6]   = left arm
  //   [7..13]  = right arm
  //   [14]     = torso
  const size_t N = std::min<std::size_t>(15, a->data.size());

  for (size_t i = 0; i < N; ++i) {
    obs_[30 + i] = a->data[i];
  }
}


void IKObsPublisher::onTimer()
{
  std_msgs::msg::Float64MultiArray m;
  m.data = obs_;
  pub_obs_->publish(m);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IKObsPublisher>());
  rclcpp::shutdown();
  return 0;
}
