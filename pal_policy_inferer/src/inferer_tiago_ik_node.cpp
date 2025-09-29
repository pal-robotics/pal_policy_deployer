#include "pal_policy_inferer/inferer_tiago_ik_node.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

// NODE CLASS
TiagoPolicyInfererIK::TiagoPolicyInfererIK()
: rclcpp::Node("pal_policy_inferer"),
  env_(ORT_LOGGING_LEVEL_WARNING, "rl"),
  mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)),
  observation_(24, 0.0)
{
  // Parameters
  this->declare_parameter<double>("start_delay", 0.5);
  this->get_parameter("start_delay", start_delay_);
  RCLCPP_INFO(this->get_logger(), "Waiting %.1f seconds before starting...", start_delay_);
  rclcpp::sleep_for(
    std::chrono::milliseconds(
      static_cast<int64_t>(start_delay_ * 1000)));

  // TF2 buffer
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Localize the ONNX model file
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

  // Set up ONNX session
  {
    Ort::SessionOptions so;
    sess_.reset(new Ort::Session(env_, model_path_.c_str(), so));
    RCLCPP_INFO(this->get_logger(), "debug 1");
    in_names_.push_back(sess_->GetInputNameAllocated(0, alloc_).release());
    out_names_.push_back(sess_->GetOutputNameAllocated(0, alloc_).release());
  }

  // Publishers & subscriber
  pub_q_desired_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "q_desired", 1);
  pub_last_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/ik_policy/last_action",
    1);


  sub_obs_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/ik_policy/obs", 1,
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
    obs_input.assign(observation_.begin(), observation_.end()); // 24 values
  }

  if (obs_input.size() >= 24) {
    const float raw_x = obs_input[21];
    const float raw_y = obs_input[22];
    const float raw_z = obs_input[23];
    try {
      geometry_msgs::msg::PointStamped p_in;
      p_in.header.stamp = this->get_clock()->now();
      p_in.header.frame_id = source_frame_;  // "base_footprint"
      p_in.point.x = static_cast<double>(obs_input[21]);
      p_in.point.y = static_cast<double>(obs_input[22]);
      p_in.point.z = static_cast<double>(obs_input[23]);

      geometry_msgs::msg::PointStamped p_out =
        tf_buffer_->transform(p_in, target_frame_, tf2::durationFromSec(0.2));


      // overwrite the obs target with the transformed point (arm frame)
      obs_input[21] = static_cast<float>(p_out.point.x);
      obs_input[22] = static_cast<float>(p_out.point.y);
      obs_input[23] = static_cast<float>(p_out.point.z);

      RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Target %s -> %s | raw=[%.3f, %.3f, %.3f]  transformed=[%.3f, %.3f, %.3f]",
      source_frame_.c_str(), target_frame_.c_str(),
      raw_x, raw_y, raw_z,
      obs_input[21], obs_input[22], obs_input[23]);
    }
    catch (const tf2::TransformException & ex) {
      // If TF isn’t ready, keep the original values (base frame) for this tick.
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "TF transform %s->%s failed: %s",
                           source_frame_.c_str(), target_frame_.c_str(), ex.what());
    }
  }

  // std::ostringstream oss;
  // oss << "[";
  // for (size_t i = 20; i < 23; ++i) {
  //   if (i) oss << ", ";
  //   oss << obs_input[i];
  // }
  // oss << "]";
  // RCLCPP_INFO(this->get_logger(), "obs: %s", oss.str().c_str());

  // ---- wrap it in an Ort::Value tensor
  static constexpr int64_t input_shape[2] = {1, 24};
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
  constexpr float action_scale = 1.0f;

  // post processing
  std::vector<double> default_pose = {0.242, 0.002, -0.073, -0.0, -0.022, -0.009, 0.004};

  std::vector<double> output = {
    default_pose[0] + action_scale * static_cast<double>(y_f[0]),
    default_pose[1] + action_scale * static_cast<double>(y_f[1]),
    default_pose[2] + action_scale * static_cast<double>(y_f[2]),
    default_pose[3] + action_scale * static_cast<double>(y_f[3]),
    default_pose[4] + action_scale * static_cast<double>(y_f[4]),
    default_pose[5] + action_scale * static_cast<double>(y_f[5]),
    default_pose[6] + action_scale * static_cast<double>(y_f[6]),
  };

  std::vector<double> min_val = {
    -3.14159265, -3.14159265, -3.14159265,
    -3.14159265, -3.14159265, -3.14159265,
    -3.14159265,
  };

  std::vector<double> max_val = {
    3.14159265, 3.14159265, 3.14159265,
    3.14159265, 3.14159265, 3.14159265,
    3.14159265,
  };

  std::vector<double> clamped_action(7);
  for (size_t i = 0; i < 7; ++i) {
    clamped_action[i] = clamp(output[i], min_val[i], max_val[i]);
  }

  std_msgs::msg::Float64MultiArray qmsg;
  qmsg.data = clamped_action;
  pub_q_desired_->publish(qmsg);

  last.data.resize(7);
  for (size_t i = 0; i < 7; ++i) {
    last.data[i] = static_cast<double>(y_f[i]);
  }
  pub_last_->publish(last);
}

void TiagoPolicyInfererIK::obsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != 24) {
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
