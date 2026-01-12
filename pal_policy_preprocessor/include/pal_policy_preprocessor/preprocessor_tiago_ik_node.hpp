#pragma once

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <onnxruntime_cxx_api.h>

// -------------------- POLICY CONFIG --------------------
struct PolicyConfig
{
  // raw metadata
  std::string run_path;
  std::vector<std::string> joint_names;          // ALL joints in obs (incl grippers)
  std::vector<double> default_joint_pos;         // same length as joint_names

  std::vector<double> action_scale;              // optional
  std::vector<double> joint_stiffness;           // optional
  std::vector<double> joint_damping;             // optional

  // derived
  std::vector<std::string> action_joint_names;   // joint_names excluding grippers
  int obs_dim{0};
  int action_dim{0};
};

class IKObsPublisher : public rclcpp::Node
{
public:
  explicit IKObsPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // -------------------- Callbacks --------------------
  void jsCB(const sensor_msgs::msg::JointState::SharedPtr msg);

  void leftPosCB(const geometry_msgs::msg::Point::SharedPtr msg);
  void leftQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg);
  void rightPosCB(const geometry_msgs::msg::Point::SharedPtr msg);
  void rightQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg);

  void lastCB(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void onTimer();

  // -------------------- Helpers --------------------
  void initBuffersFromConfig();
  void buildJointIndexMap();
  std::vector<double> buildObservationVector() const;

  // -------------------- ORT + config --------------------
  Ort::Env env_;
  std::string model_path_;
  PolicyConfig config_;

  // -------------------- ROS interfaces --------------------
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_left_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_left_quat_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_right_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_right_quat_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_last_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_obs_;
  rclcpp::TimerBase::SharedPtr timer_;

  // -------------------- State --------------------
  mutable std::mutex mtx_;
  std::unordered_map<std::string, size_t> joint_name_to_index_;

  std::vector<double> buff_joint_pos_;    // size N (relative to default)
  std::vector<double> buff_joint_vel_;    // size N
  std::vector<double> buff_last_action_;  // size M
  std::vector<double> buff_command_;      // size 14
};
