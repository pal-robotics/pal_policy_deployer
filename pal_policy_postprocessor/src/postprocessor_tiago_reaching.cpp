#include "pal_policy_postprocessor/postprocessor_tiago_reaching.hpp"

#include <algorithm>

TiagoReachingPostprocessor::TiagoReachingPostprocessor(
  const rclcpp::NodeOptions & options)
: Postprocessor("tiago_reaching_postprocessor", options)
{
  action_scale_ = this->declare_parameter<double>("action_scale", double{0.5});

  const std::string default_joint_pos_str = this->declare_parameter<std::string>(
    "default_joint_pos", "");
  default_joint_pos_ = parseDoubles(default_joint_pos_str);

  left_default_ = std::vector<double>(
    default_joint_pos_.begin() + 1,
    default_joint_pos_.begin() + 1 + 7
  );

  right_default_ = std::vector<double>(
    default_joint_pos_.begin() + 10,
    default_joint_pos_.begin() + 10 + 7
  );

  msg_left_.data.resize(left_default_.size());
  msg_right_.data.resize(right_default_.size());
  last_.data.resize(15);

  min_val_ = this->declare_parameter<std::vector<double>>(
    "minimum_joint_positions", std::vector<double>{});
  max_val_ = this->declare_parameter<std::vector<double>>(
    "maximum_joint_positions", std::vector<double>{});

  pub_q_left_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "pal_impedance_control_left/q_desired", 1);

  pub_q_right_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "pal_impedance_control_right/q_desired", 1);

  pub_last_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/reaching_tiago/last_action", 1);
}

void TiagoReachingPostprocessor::postprocessAndPublish(
  const std::vector<double> & y)
{
  // expected layout:
  // y[0]      torso
  // y[1..7]   left arm
  // y[8..14]  right arm

  for (size_t i = 0; i < 7; ++i) {
    msg_left_.data[i] =
      clamp(
      left_default_[i] + action_scale_ * y[1 + i],
      min_val_[i], max_val_[i]);

    msg_right_.data[i] =
      clamp(
      right_default_[i] + action_scale_ * y[8 + i],
      min_val_[i], max_val_[i]);
  }

  // publish desired joint positions

  pub_q_left_->publish(msg_left_);
  pub_q_right_->publish(msg_right_);
  // publish last action (raw network output)
  last_.data.assign(y.begin(), y.begin() + 15);
  pub_last_->publish(last_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TiagoReachingPostprocessor>());
  rclcpp::shutdown();
  return 0;
}
