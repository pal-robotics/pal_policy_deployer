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


class OnnxInferer : public rclcpp::Node
{
public:
  OnnxInferer();
  ~OnnxInferer() override;

  bool is_ready() const {return ready_;}

private:
  void timerCallback();
  void obsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  // ONNX Runtime
  Ort::Env env_;
  std::unique_ptr<Ort::Session> sess_;
  Ort::MemoryInfo mem_;
  Ort::AllocatorWithDefaultOptions alloc_;
  std::vector<char *> in_names_;
  std::vector<char *> out_names_;

  size_t obs_dim_{0};

  // ROS 2 pubs/subs/timer
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_output_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_obs_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Observation buffer (59-dim: actions + joint_pos + joint_vel + commands)
  std::vector<double> observation_;
  std::mutex observation_mutex_;

  // Params/state
  double start_delay_{0.5};
  std::string model_path_;
  bool ready_{false};
};
