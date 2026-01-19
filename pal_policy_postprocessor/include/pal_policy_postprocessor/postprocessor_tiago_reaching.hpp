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
  static double clamp(double v, double lo, double hi);

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_q_left_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_q_right_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_last_;
};
