#include "pal_policy_preprocessor/preprocessor_tiago_ik_node.hpp"
using std::placeholders::_1;

IKObsPublisher::IKObsPublisher()
: rclcpp::Node("onnx_input_publisher"), obs_(24, 0.0)
{
  // initial target (x, y, z)
  obs_[21] = 0.5;
  obs_[22] = 0.5;
  obs_[23] = 0.5;

  sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 1, std::bind(&IKObsPublisher::jsCB, this, _1));
  sub_pose_ = this->create_subscription<geometry_msgs::msg::Point>(
    "/target_position", 1, std::bind(&IKObsPublisher::poseCB, this, _1));
  sub_last_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/ik_policy/last_action", 1, std::bind(&IKObsPublisher::lastCB, this, _1));

  pub_obs_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/ik_policy/obs", 1);

  timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&IKObsPublisher::onTimer, this));
}


void IKObsPublisher::jsCB(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 7 joint positions
  obs_[0] = msg->position[8];
  obs_[1] = msg->position[9];
  obs_[2] = msg->position[10];
  obs_[3] = msg->position[11];
  obs_[4] = msg->position[12];
  obs_[5] = msg->position[13];
  obs_[6] = msg->position[14];

  // 7 joint velocities
  obs_[7] = msg->velocity[8];
  obs_[8] = msg->velocity[9];
  obs_[9] = msg->velocity[10];
  obs_[10] = msg->velocity[11];
  obs_[11] = msg->velocity[12];
  obs_[12] = msg->velocity[13];
  obs_[13] = msg->velocity[14];
}

void IKObsPublisher::poseCB(const geometry_msgs::msg::Point::SharedPtr msg)
{
  // target position
  obs_[21] = msg->x;
  obs_[22] = msg->y;
  obs_[23] = msg->z;
}

void IKObsPublisher::lastCB(const std_msgs::msg::Float64MultiArray::SharedPtr a)
{
  // 7 delta actions
  for (size_t i = 0; i < 7 && i < a->data.size(); ++i) {
    obs_[14 + i] = a->data[i];
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
