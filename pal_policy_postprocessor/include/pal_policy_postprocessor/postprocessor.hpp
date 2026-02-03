#pragma once

#include <vector>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace pal_policy_postprocessor
{
class Postprocessor : public rclcpp::Node
{
public:
  explicit Postprocessor(const std::string & node_name);
  virtual ~Postprocessor() = default;

protected:
  virtual void postprocessAndPublish(const std::vector<double> & y) = 0;
  static std::vector<double> parseDoubles(const std::string & str);

private:
  void outputCallback(
    const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_output_;
};
}
