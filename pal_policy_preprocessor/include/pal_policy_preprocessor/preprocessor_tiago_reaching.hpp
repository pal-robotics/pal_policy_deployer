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
  std::string run_path;

  std::vector<std::string> joint_names;
  std::vector<double> default_joint_pos;

  std::vector<double> action_scale;
  std::vector<double> joint_stiffness;
  std::vector<double> joint_damping;

  std::vector<std::string> action_joint_names;

  int obs_dim{0};
  int action_dim{0};
};

class TiagoReaching : public Preprocessor
{
public:
  explicit TiagoReaching(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  void buildObservationVector(std::vector<double> * obs) const override;

private:
  void parseMetadataIntoConfig(const std::map<std::string, std::string> & mm);
  void initBuffersFromConfig();
  void buildJointIndexMap();

  // callbacks
  void jsCB(const sensor_msgs::msg::JointState::SharedPtr msg);
  void leftPosCB(const geometry_msgs::msg::Point::SharedPtr msg);
  void leftQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg);
  void rightPosCB(const geometry_msgs::msg::Point::SharedPtr msg);
  void rightQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg);
  void lastCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

private:
  PolicyConfig config_;

  std::vector<double> buff_joint_pos_;
  std::vector<double> buff_joint_vel_;
  std::vector<double> buff_last_action_;
  std::vector<double> buff_command_;

  std::unordered_map<std::string, size_t> joint_name_to_index_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_left_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_left_quat_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_right_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_right_quat_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_last_;
};
