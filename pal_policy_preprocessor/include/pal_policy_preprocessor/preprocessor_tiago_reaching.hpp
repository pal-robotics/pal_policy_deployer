#pragma once

#include "preprocessor.hpp"

#include <unordered_map>
#include <vector>
#include <string>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

struct PolicyConfig
{
  std::vector<std::string> joint_names;
  std::vector<double> default_joint_pos;

  int input_dim{0};
  int output_dim{0};
};

class TiagoReachingPreprocessor : public Preprocessor
{
public:
  explicit TiagoReachingPreprocessor();

protected:
  void buildInputVector(std::vector<double> * obs) const override;

private:
  void initBuffersFromConfig();
  void buildJointIndexMap();

  // callbacks
  void jsCB(const sensor_msgs::msg::JointState::SharedPtr msg);
  void leftPosCB(const geometry_msgs::msg::Point::SharedPtr msg);
  void leftQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg);
  void rightPosCB(const geometry_msgs::msg::Point::SharedPtr msg);
  void rightQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg);
  void lastCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  PolicyConfig config_;

  std::vector<double> buff_joint_pos_;
  std::vector<double> buff_joint_vel_;
  std::vector<double> buff_last_action_;
  std::vector<double> buff_command_;

  std::vector<double> left_pos_, left_quat_, right_pos_, right_quat_;

  std::unordered_map<std::string, size_t> joint_name_to_index_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_left_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_left_quat_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_right_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_right_quat_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_last_;
};
