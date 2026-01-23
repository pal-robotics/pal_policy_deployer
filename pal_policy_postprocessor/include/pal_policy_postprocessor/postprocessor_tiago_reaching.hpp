#pragma once

#include "pal_policy_postprocessor/postprocessor.hpp"

#include <vector>

class TiagoReachingPostprocessor : public Postprocessor
{
public:
  explicit TiagoReachingPostprocessor(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void postprocessAndPublish(const std::vector<double> & y) override;

private:
  template<class T>
  static constexpr const T & clamp(const T & v, const T & lo, const T & hi)
  {
    return v < lo ? lo : (hi < v ? hi : v);
  }

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_q_left_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_q_right_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_last_;

  std_msgs::msg::Float64MultiArray msg_left_;
  std_msgs::msg::Float64MultiArray msg_right_;
  std_msgs::msg::Float64MultiArray last_;

  std::vector<double> left_default_, right_default_, default_joint_pos_, min_val_, max_val_;
  double action_scale_;
};
