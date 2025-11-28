#include "pal_policy_inferer/inferer_tiago_ik_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace fs = std::filesystem;

TiagoPolicyInfererIK::TiagoPolicyInfererIK()
: rclcpp::Node("pal_policy_inferer"),
  env_(ORT_LOGGING_LEVEL_WARNING, "rl"),
  mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
  observation_(59, 0.0)
{
  this->declare_parameter<double>("start_delay", 0.5);
  this->get_parameter("start_delay", start_delay_);
  RCLCPP_INFO(this->get_logger(), "Waiting %.1f seconds before starting...", start_delay_);
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(start_delay_ * 1000)));

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

  pub_q_desired_right_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "pal_impedance_control_right/q_desired", 1);
  pub_q_desired_left_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "pal_impedance_control_left/q_desired", 1);
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

  static constexpr int64_t input_shape[2] = {1, 59};
  Ort::Value input_tensor =
    Ort::Value::CreateTensor<float>(mem_, obs_input.data(), obs_input.size(), input_shape, 2);

  auto outputs = sess_->Run(
    Ort::RunOptions{nullptr},
    const_cast<const char * const *>(in_names_.data()), &input_tensor, 1,
    const_cast<const char * const *>(out_names_.data()), 1);

  const float * y_f = outputs.front().GetTensorData<float>();

  std_msgs::msg::Float64MultiArray last;
  constexpr float action_scale = 0.5f;

  double q_torso = static_cast<double>(obs_input[0]);
  double torso_cmd = action_scale * static_cast<double>(y_f[0]);

  std::array<double, 7> q_left{};
  std::array<double, 7> q_right{};

  for (size_t i = 0; i < 7; ++i) {
    q_left[i] = static_cast<double>(obs_input[1 + i]);
    q_right[i] = static_cast<double>(obs_input[8 + i]);
  }

  std::vector<double> left_cmd(7);
  std::vector<double> right_cmd(7);

  for (size_t i = 0; i < 7; ++i) {
    left_cmd[i] = action_scale * static_cast<double>(y_f[1 + i]);    // y_f[1..7]
    right_cmd[i] = action_scale * static_cast<double>(y_f[8 + i]);   // y_f[8..14]
  }

  std::vector<double> min_val = {
    -4.71239, -2.44346, -2.61799, -2.44346, -1.5708, -1.88496, -2.61799,
  };

  std::vector<double> max_val = {
    0.523599, 1.13446, 2.61799, 1.13446, 3.66519, 3.00197, 2.61799,
  };

  std::vector<double> clamped_left(7);
  std::vector<double> clamped_right(7);

  for (size_t i = 0; i < 7; ++i) {
    clamped_left[i] = clamp(left_cmd[i], min_val[i], max_val[i]);
    clamped_right[i] = clamp(right_cmd[i], min_val[i], max_val[i]);
  }

  std_msgs::msg::Float64MultiArray qmsg_left;
  qmsg_left.data = clamped_left;
  pub_q_desired_left_->publish(qmsg_left);

  std_msgs::msg::Float64MultiArray qmsg_right;
  qmsg_right.data = clamped_right;
  pub_q_desired_right_->publish(qmsg_right);

  // Publish last actions: torso + 7 left + 7 right = 15
  // (pure deltas, without current position)
  last.data.resize(15);

  // Torso
  last.data[0] = torso_cmd;

  // Left arm
  for (size_t i = 0; i < 7; ++i) {
    last.data[1 + i] = left_cmd[i];
  }

  // Right arm
  for (size_t i = 0; i < 7; ++i) {
    last.data[8 + i] = right_cmd[i];
  }

  pub_last_->publish(last);
}

void TiagoPolicyInfererIK::obsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 59) {
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
