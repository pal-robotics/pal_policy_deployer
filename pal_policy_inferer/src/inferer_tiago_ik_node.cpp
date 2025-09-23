#include "pal_policy_inferer/inferer_tiago_ik_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

// NODE CLASS
TiagoPolicyInfererIK::TiagoPolicyInfererIK()
: rclcpp::Node("pal_policy_inferer"),
  env_(ORT_LOGGING_LEVEL_WARNING, "rl"),
  mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
  observation_(65, 0.0)
{
  // Parameters
  this->declare_parameter<double>("start_delay", 0.5);
  this->get_parameter("start_delay", start_delay_);
  RCLCPP_INFO(this->get_logger(), "Waiting %.1f seconds before starting...", start_delay_);
  rclcpp::sleep_for(
    std::chrono::milliseconds(
      static_cast<int64_t>(start_delay_ * 1000)));

  // Localize the ONNX model file
  std::string pkg_share = ament_index_cpp::get_package_share_directory("pal_policy_deployer");
  std::string config_dir = (fs::path(pkg_share) / "models" / "reach" / "exported").string();

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

  // Set up ONNX session
  {
    Ort::SessionOptions so;
    sess_.reset(new Ort::Session(env_, model_path_.c_str(), so));

    in_names_.push_back(sess_->GetInputNameAllocated(0, alloc_).release());
    out_names_.push_back(sess_->GetOutputNameAllocated(0, alloc_).release());
  }

  // Publishers & subscriber
  pub_traj_cmd_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/left_leg_controller/command", 1);
  pub_sub_cmd_ = this->create_publisher<sensor_msgs::msg::JointState>(
    "/subscriber_controller/desired_state", 1);
  pub_last_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/reach_policy/last_action",
    1);

  sub_obs_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/reach_policy/obs", 1,
    std::bind(&TiagoPolicyInfererIK::obsCallback, this, std::placeholders::_1));

  // Timer callback 50Hz
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

  // ---- copy the latest observation under lock
  std::vector<float> obs_input;
  {
    std::lock_guard<std::mutex> lk(observation_mutex_);
    obs_input.assign(observation_.begin(), observation_.end()); // 65 values
  }

  // ---- wrap it in an Ort::Value tensor
  static constexpr int64_t input_shape[2] = {1, 65};
  Ort::Value input_tensor =
    Ort::Value::CreateTensor<float>(mem_, obs_input.data(), obs_input.size(), input_shape, 2);

  // ---- run inference
  auto outputs = sess_->Run(
    Ort::RunOptions{nullptr},
    const_cast<const char * const *>(in_names_.data()), &input_tensor, 1,
    const_cast<const char * const *>(out_names_.data()), 1);

  const float * y_f = outputs.front().GetTensorData<float>();

  // ---- publish results
  std_msgs::msg::Float64MultiArray last;
  constexpr float action_scale = 0.5f;

  std::vector<double> action = {
    action_scale * static_cast<double>(y_f[0]),
    action_scale * static_cast<double>(y_f[1]) + 0.05,
    action_scale * static_cast<double>(y_f[2]),
    action_scale * static_cast<double>(y_f[4]) - 0.05,
    action_scale * static_cast<double>(y_f[5]),
    action_scale * static_cast<double>(y_f[3]) + 0.6832,
  };

  std::vector<double> min_val = {
    -0.2617993877991494,
    -0.7417649320975901,
    -0.47123889803846897,
    0.131973,
    -0.7417649,
    -0.4712389,
  };

  std::vector<double> max_val = {
    0.6981317007977318,
    0.6632251157578453,
    0.47123889803846897,
    0.71416639,
    0.6632251,
    0.4712389
  };

  std::vector<double> clamped_action(6);
  for (size_t i = 0; i < 6; ++i) {
    clamped_action[i] = clamp(action[i], min_val[i], max_val[i]);
  }

  trajectory_msgs::msg::JointTrajectory traj;
  traj.header.stamp = this->get_clock()->now();
  traj.joint_names = {
    "leg_left_1_joint",
    "leg_left_2_joint",
    "leg_left_3_joint",
    "leg_left_4_joint",
    "leg_left_5_joint",
    "leg_left_length_joint",
  };

  trajectory_msgs::msg::JointTrajectoryPoint pt;
  pt.positions = {
    clamped_action[0],
    clamped_action[1],
    clamped_action[2],
    clamped_action[3],
    clamped_action[4],
    clamped_action[5],
  };
  pt.time_from_start.sec = 1;
  pt.time_from_start.nanosec = 0;

  traj.points.push_back(pt);
  pub_traj_cmd_->publish(traj);

  last.data.resize(6);
  for (size_t i = 0; i < 6; ++i) {
    last.data[i] = static_cast<double>(y_f[i]);
  }
  pub_last_->publish(last);
}

void TiagoPolicyInfererIK::obsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 65) {
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
    // Match original style: log fatal earlier, just exit with non-zero.
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
