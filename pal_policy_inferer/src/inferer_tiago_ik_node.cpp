#include "pal_policy_inferer/inferer_tiago_ik_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace fs = std::filesystem;

TiagoPolicyInfererIK::TiagoPolicyInfererIK()
: rclcpp::Node("pal_policy_inferer"),
  env_(ORT_LOGGING_LEVEL_WARNING, "rl"),
  mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
  observation_(28, 0.0)
{
  this->declare_parameter<double>("start_delay", 0.5);
  this->get_parameter("start_delay", start_delay_);
  RCLCPP_INFO(this->get_logger(), "Waiting %.1f seconds before starting...", start_delay_);
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(start_delay_ * 1000)));

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  static_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

  const double cutoff_hz = 2.0;
  const double dt = 0.02;
  for (size_t i = 0; i < 7; ++i) {
    joint_filters_[i] = LowPassFilter(cutoff_hz, dt);
  }

  try {
    (void)tf_buffer_->lookupTransform(
      source_frame_,
      target_frame_,
      tf2::TimePointZero,
      tf2::durationFromSec(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_FATAL(
      this->get_logger(), "Failed to capture TF %s -> %s: %s",
      source_frame_.c_str(), target_frame_.c_str(), ex.what());
    return;
  }

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

  {
    Ort::SessionOptions so;
    sess_.reset(new Ort::Session(env_, model_path_.c_str(), so));
    in_names_.push_back(sess_->GetInputNameAllocated(0, alloc_).release());
    out_names_.push_back(sess_->GetOutputNameAllocated(0, alloc_).release());
  }

  pub_q_desired_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "pal_impedance_control_right/q_desired", 1);
  pub_last_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/ik_policy/last_action", 1);

  sub_obs_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/ik_policy/obs", 1,
    std::bind(&TiagoPolicyInfererIK::obsCallback, this, std::placeholders::_1));

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&TiagoPolicyInfererIK::timerCallback, this));

  ready_ = true;
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

  std::vector<float> obs_input;
  {
    std::lock_guard<std::mutex> lk(observation_mutex_);
    obs_input.assign(observation_.begin(), observation_.end());
  }

  if (obs_input.size() >= 28) {
    const float raw_x = obs_input[21];
    const float raw_y = obs_input[22];
    const float raw_z = obs_input[23];
    try {
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

      geometry_msgs::msg::PoseStamped pose_in;
      pose_in.header = p_in.header;
      pose_in.pose.position.x = 0.0;
      pose_in.pose.position.y = 0.0;
      pose_in.pose.position.z = 0.0;

      const double qw = static_cast<double>(obs_input[24]);
      const double qx = static_cast<double>(obs_input[25]);
      const double qy = static_cast<double>(obs_input[26]);
      const double qz = static_cast<double>(obs_input[27]);
      pose_in.pose.orientation.x = qx;
      pose_in.pose.orientation.y = qy;
      pose_in.pose.orientation.z = qz;
      pose_in.pose.orientation.w = qw;

      geometry_msgs::msg::PoseStamped pose_out =
        tf_buffer_->transform(pose_in, target_frame_, tf2::durationFromSec(0.2));

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
    }
  }

  static constexpr int64_t input_shape[2] = {1, 28};
  Ort::Value input_tensor =
    Ort::Value::CreateTensor<float>(mem_, obs_input.data(), obs_input.size(), input_shape, 2);

  auto outputs = sess_->Run(
    Ort::RunOptions{nullptr},
    const_cast<const char * const *>(in_names_.data()), &input_tensor, 1,
    const_cast<const char * const *>(out_names_.data()), 1);

  const float * y_f = outputs.front().GetTensorData<float>();

  std_msgs::msg::Float64MultiArray last;
  constexpr float action_scale = 0.5f;

  std::array<double, 7> q{};
  for (size_t i = 0; i < 7; ++i) {
    q[i] = static_cast<double>(obs_input[i]);
  }

  std::vector<double> default_pose = {0.36, -1.83, -0.47, -2.35, -0.0, -1.2, 0.0};

  std::vector<double> output = {
    q[0] + action_scale * static_cast<double>(y_f[0]),
    q[1] + action_scale * static_cast<double>(y_f[1]),
    q[2] + action_scale * static_cast<double>(y_f[2]),
    q[3] + action_scale * static_cast<double>(y_f[3]),
    q[4] + action_scale * static_cast<double>(y_f[4]),
    q[5] + action_scale * static_cast<double>(y_f[5]),
    q[6] + action_scale * static_cast<double>(y_f[6]),
  };

  std::vector<double> min_val = {
    -4.71239, -2.44346, -2.61799, -2.44346, -1.5708, -1.88496, -2.61799,
  };

  std::vector<double> max_val = {
    0.523599, 1.13446, 2.61799, 1.13446, 3.66519, 3.00197, 2.61799,
  };

  std::vector<double> clamped_action(7);
  for (size_t i = 0; i < 7; ++i) {
    clamped_action[i] = clamp(output[i], min_val[i], max_val[i]);
  }

  std_msgs::msg::Float64MultiArray qmsg;
  qmsg.data = clamped_action;
  std::vector<double> filtered_action(7);
  for (size_t i = 0; i < 7; ++i) {
    filtered_action[i] = joint_filters_[i].update(static_cast<float>(clamped_action[i]));
  }
  qmsg.data = filtered_action;
  pub_q_desired_->publish(qmsg);

  last.data.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    last.data[i] = action_scale * static_cast<double>(y_f[i]);
  }

  pub_last_->publish(last);
}

void TiagoPolicyInfererIK::obsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 28) {
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
