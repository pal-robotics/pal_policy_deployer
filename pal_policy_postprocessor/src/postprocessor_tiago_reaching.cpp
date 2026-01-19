#include "pal_policy_postprocessor/postprocessor_tiago_reaching.hpp"

#include <algorithm>

TiagoReachingPostprocessor::TiagoReachingPostprocessor(
  const rclcpp::NodeOptions & options)
: Postprocessor("tiago_reaching_postprocessor", options)
{
  pub_q_left_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "pal_impedance_control_left/q_desired", 1);

  pub_q_right_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "pal_impedance_control_right/q_desired", 1);

  pub_last_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/reaching_tiago/last_action", 1);
}

double TiagoReachingPostprocessor::clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

void TiagoReachingPostprocessor::postprocessAndPublish(
  const std::vector<double> & y)
{
  // expected layout:
  // y[0]      torso
  // y[1..7]   left arm
  // y[8..14]  right arm

  if (y.size() < 15) {
    RCLCPP_WARN(
      this->get_logger(),
      "Postprocessor: action vector too small (%zu)", y.size());
    return;
  }

  constexpr double action_scale = 0.5;

  const std::vector<double> left_default =
  {0.358, -1.827, 0.470, -2.341, 0.000, -1.201, 0.000};
  const std::vector<double> right_default =
  {-0.358, -1.827, -0.470, -2.341, 0.000, -1.201, 0.000};

  const std::vector<double> min_val =
  {-4.71239, -2.44346, -2.61799, -2.44346, -1.5708, -1.88496, -2.61799};
  const std::vector<double> max_val =
  {0.523599, 1.13446, 2.61799, 1.13446, 3.66519, 3.00197, 2.61799};

  std::vector<double> left_cmd(7);
  std::vector<double> right_cmd(7);

  for (size_t i = 0; i < 7; ++i) {
    left_cmd[i] =
      clamp(
      left_default[i] + action_scale * y[1 + i],
      min_val[i], max_val[i]);

    right_cmd[i] =
      clamp(
      right_default[i] + action_scale * y[8 + i],
      min_val[i], max_val[i]);
  }

  // publish desired joint positions
  std_msgs::msg::Float64MultiArray msg_left;
  msg_left.data = left_cmd;
  pub_q_left_->publish(msg_left);

  std_msgs::msg::Float64MultiArray msg_right;
  msg_right.data = right_cmd;
  pub_q_right_->publish(msg_right);

  // publish last action (raw network output)
  std_msgs::msg::Float64MultiArray last;
  last.data.assign(y.begin(), y.begin() + 15);
  pub_last_->publish(last);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TiagoReachingPostprocessor>());
  rclcpp::shutdown();
  return 0;
}
