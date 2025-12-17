#include "pal_policy_preprocessor/preprocessor_tiago_ik_node.hpp"

#include <algorithm>
#include <chrono>
#include <string>
#include <vector>

using std::placeholders::_1;

// ---- OBSERVATION JOINT ORDER (same style as ROS1) ----
// 15 joints total: torso + left(7) + right(7)
static const std::vector<std::string> obs_order = {
  "torso_lift_joint",
  "arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint", "arm_left_4_joint",
  "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint",
  "arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint", "arm_right_4_joint",
  "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint"
};

// ---- DEFAULT JOINT POSITIONS ----
static const double torso_default = 0.1;

static const std::vector<double> left_default = {
  0.358, -1.827, 0.470, -2.341, 0.000, -1.201, 0.000
};

static const std::vector<double> right_default = {
  -0.358, -1.827, -0.470, -2.341, 0.000, -1.201, 0.000
};

IKObsPublisher::IKObsPublisher()
: rclcpp::Node("onnx_input_publisher"), obs_(59, 0.0)
{
  // === Initial left command pose (xyz, qw, qx, qy, qz) ===
  obs_[45] = 0.337962;
  obs_[46] = 0.203562;
  obs_[47] = 0.677366;
  obs_[48] = 0.078844;
  obs_[49] = 0.21089;
  obs_[50] = 0.00096864;
  obs_[51] = 0.97432;

  // === Initial right command pose (xyz, qw, qx, qy, qz) ===
  obs_[52] = 0.337751;
  obs_[53] = -0.202208;
  obs_[54] = 0.677405;
  obs_[55] = -0.078344;
  obs_[56] = 0.21066;
  obs_[57] = 0.0041078;
  obs_[58] = 0.97441;

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

  pub_obs_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/ik_policy/obs", 1);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&IKObsPublisher::onTimer, this));
}

// JointState callback
void IKObsPublisher::jsCB(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->name.empty() || msg->position.size() != msg->name.size()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "JointState invalid");
    return;
  }

  for (size_t k = 0; k < obs_order.size(); ++k) {
    const auto & wanted = obs_order[k];

    for (size_t i = 0; i < msg->name.size(); ++i) {
      if (msg->name[i] == wanted) {

        double q = msg->position[i];

        // ---- subtract default pose ----
        if (k == 0) {
          q -= torso_default;
        } else if (k >= 1 && k <= 7) {
          q -= left_default[k - 1];
        } else if (k >= 8 && k <= 14) {
          q -= right_default[k - 8];
        }

        obs_[k] = q;

        if (!msg->velocity.empty()) {
          obs_[15 + k] = msg->velocity[i];
        }
        break;
      }
    }
  }
}

// ---- Target pose callbacks ----
void IKObsPublisher::leftPosCB(const geometry_msgs::msg::Point::SharedPtr msg)
{
  obs_[45] = msg->x;
  obs_[46] = msg->y;
  obs_[47] = msg->z;
}

void IKObsPublisher::leftQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg)
{
  obs_[48] = msg->w;
  obs_[49] = msg->x;
  obs_[50] = msg->y;
  obs_[51] = msg->z;
}

void IKObsPublisher::rightPosCB(const geometry_msgs::msg::Point::SharedPtr msg)
{
  obs_[52] = msg->x;
  obs_[53] = msg->y;
  obs_[54] = msg->z;
}

void IKObsPublisher::rightQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg)
{
  obs_[55] = msg->w;
  obs_[56] = msg->x;
  obs_[57] = msg->y;
  obs_[58] = msg->z;
}

void IKObsPublisher::lastCB(const std_msgs::msg::Float64MultiArray::SharedPtr a)
{
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
