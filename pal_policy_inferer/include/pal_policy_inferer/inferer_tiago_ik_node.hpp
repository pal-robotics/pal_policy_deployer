#pragma once

#include <memory>
#include <mutex>
#include <vector>
#include <algorithm>
#include <chrono>
#include <thread>
#include <iostream>
#include <filesystem>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <onnxruntime_cxx_api.h>

// TF2
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/static_transform_broadcaster.h>

class TiagoPolicyInfererIK : public rclcpp::Node
{
public:
  TiagoPolicyInfererIK();
  ~TiagoPolicyInfererIK() override;

  bool is_ready() const {return ready_;}

private:
  template<class T>
  static constexpr const T & clamp(const T & v, const T & lo, const T & hi)
  {
    return v < lo ? lo : (hi < v ? hi : v);
  }

  void timerCallback();
  void obsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // ONNX Runtime
  Ort::Env env_;
  std::unique_ptr<Ort::Session> sess_;
  Ort::MemoryInfo mem_;
  Ort::AllocatorWithDefaultOptions alloc_;
  std::vector<char *> in_names_;
  std::vector<char *> out_names_;

  // ROS 2 pubs/subs/timer
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_q_desired_right_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_q_desired_left_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_last_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_obs_;
  rclcpp::TimerBase::SharedPtr timer_;

  // TF2 (for transforming command poses, e.g. base_footprint -> base_link)
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  std::string source_frame_{"base_footprint"};
  std::string target_frame_{"base_link"};

  // Observation buffer (59-dim: actions + joint_pos + joint_vel + commands)
  std::vector<double> observation_;
  std::mutex observation_mutex_;

  // Params/state
  double start_delay_{0.5};
  std::string model_path_;
  bool ready_{false};
};
