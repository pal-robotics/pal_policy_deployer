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

  std::vector<double> left_cmd_;
  std::vector<double> right_cmd_;

  std::vector<double> left_default_, right_default_, min_val_, max_val_;
  double action_scale_;
};
