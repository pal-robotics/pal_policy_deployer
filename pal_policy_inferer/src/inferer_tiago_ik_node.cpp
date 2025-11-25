// SPDX-License-Identifier: Apache-2.0
// Copyright 2025
//
// File: inferer_tiago_ik_node.cpp
//
// A ROS 2 node that:
//  - receives a 28-float observation
//  - transforms target position (idx 21..23) and orientation (idx 24..27, wxyz)
//    from source_frame_ to target_frame_ using TF2
//  - runs ONNX policy inference
//  - clamps and low-pass-filters the joint targets
//  - publishes desired joint positions for impedance control

#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <onnxruntime_cxx_api.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

// --------------------- small helpers ---------------------
template<typename T>
inline T clamp(const T & v, const T & lo, const T & hi)
{
  return std::min(std::max(v, lo), hi);
}

struct LowPassFilter
{
  LowPassFilter() = default;
  LowPassFilter(double cutoff_hz, double dt_sec) {reset(cutoff_hz, dt_sec);}
  void reset(double cutoff_hz, double dt_sec, double init = 0.0)
  {
    // First-order low-pass: alpha = dt / (dt + 1/(2*pi*fc))
    const double RC = 1.0 / (2.0 * M_PI * std::max(1e-6, cutoff_hz));
    alpha_ = dt_sec / (dt_sec + RC);
    y_ = init;
    initialized_ = false;
  }
  double update(double x)
  {
    if (!initialized_) {y_ = x; initialized_ = true; return y_;}
    y_ = y_ + alpha_ * (x - y_);
    return y_;
  }

private:
  double alpha_{1.0};
  double y_{0.0};
  bool initialized_{false};
};

// =================== Node ===================
class TiagoPolicyInfererIK : public rclcpp::Node
{
public:
  TiagoPolicyInfererIK();
  ~TiagoPolicyInfererIK();

  bool is_ready() const {return ready_;}

private:
  void timerCallback();
  void obsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

private:
  // --- ONNX Runtime ---
  Ort::Env env_;
  Ort::MemoryInfo mem_;
  Ort::AllocatorWithDefaultOptions alloc_;
  std::unique_ptr<Ort::Session> sess_;
  std::vector<char *> in_names_;
  std::vector<char *> out_names_;
  std::string model_path_;

  // --- ROS I/O ---
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_obs_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_q_desired_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_last_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- TF2 ---
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  // --- state ---
  std::vector<float> observation_;
  std::mutex observation_mutex_;
  bool ready_{false};
  double start_delay_{0.5};

  // frames (parameters)
  std::string source_frame_{"base_footprint"};
  std::string target_frame_{"arm_right_root_link"};

  // filters
  std::array<LowPassFilter, 7> joint_filters_;
};

// ---------------- Implementation ----------------

TiagoPolicyInfererIK::TiagoPolicyInfererIK()
: rclcpp::Node("pal_policy_inferer"),
  env_(ORT_LOGGING_LEVEL_WARNING, "rl"),
  mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
  observation_(28, 0.0f)
{
  // Parameters
  this->declare_parameter<double>("start_delay", start_delay_);
  this->get_parameter("start_delay", start_delay_);
  this->declare_parameter<std::string>("source_frame", source_frame_);
  this->declare_parameter<std::string>("target_frame", target_frame_);
  this->get_parameter("source_frame", source_frame_);
  this->get_parameter("target_frame", target_frame_);

  RCLCPP_INFO(this->get_logger(), "Waiting %.1fs before starting...", start_delay_);
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(start_delay_ * 1000)));

  // TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  // Filters
  const double cutoff_hz = 2.0;
  const double dt = 0.02; // 50 Hz
  for (auto & f : joint_filters_) {
    f = LowPassFilter(cutoff_hz, dt);
  }

  // Sanity-check TF link (source -> target)
  try {
    (void)tf_buffer_->lookupTransform(
      target_frame_,     // target
      source_frame_,     // source
      tf2::TimePointZero,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_FATAL(
      this->get_logger(), "TF %s -> %s not available: %s",
      source_frame_.c_str(), target_frame_.c_str(), ex.what());
    return;
  }

  // Locate ONNX model
  std::string pkg_share = ament_index_cpp::get_package_share_directory("pal_policy_deployer");
  std::string config_dir = (fs::path(pkg_share) / "models" / "tiagopro" / "exported").string();

  bool found = false;
  if (fs::exists(config_dir) && fs::is_directory(config_dir)) {
    for (const auto & entry : fs::directory_iterator(config_dir)) {
      const auto & p = entry.path();
      if (entry.is_regular_file() && p.extension() == ".onnx") {
        model_path_ = p.string();
        found = true;
        RCLCPP_INFO(this->get_logger(), "Found ONNX model: %s", model_path_.c_str());
        break;
      }
    }
  }
  if (!found) {
    RCLCPP_FATAL(this->get_logger(), "No ONNX model found in %s.", config_dir.c_str());
    return;
  }

  // ONNX session
  {
    Ort::SessionOptions so;
    sess_.reset(new Ort::Session(env_, model_path_.c_str(), so));
    in_names_.push_back(sess_->GetInputNameAllocated(0, alloc_).release());
    out_names_.push_back(sess_->GetOutputNameAllocated(0, alloc_).release());
  }

  // pubs/subs
  pub_q_desired_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "pal_impedance_control_right/q_desired", 1);
  pub_last_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/ik_policy/last_action", 1);

  sub_obs_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/ik_policy/obs", 1,
    std::bind(&TiagoPolicyInfererIK::obsCallback, this, std::placeholders::_1));

  // 50 Hz timer
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&TiagoPolicyInfererIK::timerCallback, this));

  ready_ = true;
  RCLCPP_INFO(
    this->get_logger(), "pal_policy_inferer ready. source_frame=%s target_frame=%s",
    source_frame_.c_str(), target_frame_.c_str());
}

TiagoPolicyInfererIK::~TiagoPolicyInfererIK()
{
  for (auto * p : in_names_) {
    if (p) {
      alloc_.Free(p);
    }
  }
  for (auto * p : out_names_) {
    if (p) {
      alloc_.Free(p);
    }
  }
}

void TiagoPolicyInfererIK::timerCallback()
{
  if (!ready_) {return;}

  // copy latest observation
  std::vector<float> obs_input;
  {
    std::lock_guard<std::mutex> lk(observation_mutex_);
    obs_input.assign(observation_.begin(), observation_.end());
  }
  if (obs_input.size() != 28) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Observation size %zu != 28", obs_input.size());
    return;
  }

  // ---- TF transform for target position (21..23) and orientation (24..27 wxyz) ----
  try {
    // Position
    geometry_msgs::msg::PointStamped p_in;
    p_in.header.stamp = this->get_clock()->now();
    p_in.header.frame_id = source_frame_;
    p_in.point.x = static_cast<double>(obs_input[21]);
    p_in.point.y = static_cast<double>(obs_input[22]);
    p_in.point.z = static_cast<double>(obs_input[23]);

    geometry_msgs::msg::PointStamped p_out =
      tf_buffer_->transform(p_in, target_frame_, tf2::durationFromSec(0.2));
    obs_input[21] = static_cast<float>(p_out.point.x);
    obs_input[22] = static_cast<float>(p_out.point.y);
    obs_input[23] = static_cast<float>(p_out.point.z);

    // Orientation — wrap as a Pose so tf2 rotates quaternion
    geometry_msgs::msg::PoseStamped pose_in;
    pose_in.header = p_in.header;
    pose_in.pose.position.x = 0.0;
    pose_in.pose.position.y = 0.0;
    pose_in.pose.position.z = 0.0;

    // model stores wxyz -> convert to ROS xyzw
    const double qw_in = static_cast<double>(obs_input[24]);
    const double qx_in = static_cast<double>(obs_input[25]);
    const double qy_in = static_cast<double>(obs_input[26]);
    const double qz_in = static_cast<double>(obs_input[27]);

    pose_in.pose.orientation.x = qx_in;
    pose_in.pose.orientation.y = qy_in;
    pose_in.pose.orientation.z = qz_in;
    pose_in.pose.orientation.w = qw_in;

    geometry_msgs::msg::PoseStamped pose_out =
      tf_buffer_->transform(pose_in, target_frame_, tf2::durationFromSec(0.2));

    // back to model order wxyz
    const auto & q = pose_out.pose.orientation;
    obs_input[24] = static_cast<float>(q.w);
    obs_input[25] = static_cast<float>(q.x);
    obs_input[26] = static_cast<float>(q.y);
    obs_input[27] = static_cast<float>(q.z);

  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "TF transform %s->%s failed: %s",
      source_frame_.c_str(), target_frame_.c_str(), ex.what());
    // Keep original values for this tick if TF not ready.
  }

  // ---- prepare ONNX input tensor
  static constexpr int64_t input_shape[2] = {1, 28};
  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
    mem_, obs_input.data(), obs_input.size(), input_shape, 2);

  // ---- run inference
  auto outputs = sess_->Run(
    Ort::RunOptions{nullptr},
    const_cast<const char * const *>(in_names_.data()), &input_tensor, 1,
    const_cast<const char * const *>(out_names_.data()), 1);

  const float * y_f = outputs.front().GetTensorData<float>();

  // ---- build desired joint positions (current q + scaled delta)
  // current joint positions are obs_input[0..6]
  constexpr float action_scale = 0.5f;
  std::array<double, 7> q{};
  for (size_t i = 0; i < 7; ++i) {
    q[i] = static_cast<double>(obs_input[i]);
  }

  std::vector<double> output(7);
  for (size_t i = 0; i < 7; ++i) {
    output[i] = q[i] + action_scale * static_cast<double>(y_f[i]);
  }

  // joint limits
  static const std::vector<double> min_val = {
    -4.71239, -2.44346, -2.61799, -2.44346, -1.5708, -1.88496, -2.61799,
  };
  static const std::vector<double> max_val = {
    0.523599, 1.13446, 2.61799, 1.13446, 3.66519, 3.00197, 2.61799,
  };

  std::vector<double> clamped_action(7);
  for (size_t i = 0; i < 7; ++i) {
    clamped_action[i] = clamp(output[i], min_val[i], max_val[i]);
  }

  // low-pass filter
  std::vector<double> filtered_action(7);
  for (size_t i = 0; i < 7; ++i) {
    filtered_action[i] = joint_filters_[i].update(clamped_action[i]);
  }

  // publish desired q
  std_msgs::msg::Float64MultiArray qmsg;
  qmsg.data = filtered_action;
  pub_q_desired_->publish(qmsg);

  // publish last (delta) for debugging
  std_msgs::msg::Float64MultiArray last;
  last.data.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    last.data[i] = action_scale * static_cast<double>(y_f[i]);
  }
  pub_last_->publish(last);
}

void TiagoPolicyInfererIK::obsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 28) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Dropped obs with size %zu (expected 28)", msg->data.size());
    return;
  }
  std::lock_guard<std::mutex> lk(observation_mutex_);
  observation_.assign(msg->data.begin(), msg->data.end());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TiagoPolicyInfererIK>();
  if (!node->is_ready()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
