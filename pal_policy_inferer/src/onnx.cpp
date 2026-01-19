#include "pal_policy_inferer/onnx.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;

OnnxInferer::OnnxInferer()
: rclcpp::Node("pal_policy_inferer"),
  env_(ORT_LOGGING_LEVEL_WARNING, "rl"),
  mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
  this->declare_parameter<double>("start_delay", 0.5);
  this->get_parameter("start_delay", start_delay_);
  RCLCPP_INFO(this->get_logger(), "Waiting %.1f seconds before starting...", start_delay_);
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(start_delay_ * 1000)));

  model_path_ = this->declare_parameter<std::string>("onnx_model_path", "");
  if (model_path_.empty()) {
    throw std::runtime_error("onnx_model_path param not set");
  }

  {
    Ort::SessionOptions so;
    sess_.reset(new Ort::Session(env_, model_path_.c_str(), so));

    Ort::TypeInfo input_info = sess_->GetInputTypeInfo(0);
    auto tensor_info = input_info.GetTensorTypeAndShapeInfo();

    std::vector<int64_t> dims = tensor_info.GetShape();
    if (dims.size() < 2 || dims[1] <= 0) {
      throw std::runtime_error("ONNX obs dim is dynamic/invalid (expected fixed [1, N])");
    }
    obs_dim_ = static_cast<size_t>(dims[1]);

    std::lock_guard<std::mutex> lk(observation_mutex_);
    observation_.assign(obs_dim_, 0.0);

    in_names_.push_back(sess_->GetInputNameAllocated(0, alloc_).release());
    out_names_.push_back(sess_->GetOutputNameAllocated(0, alloc_).release());
  }

  sub_obs_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/onnx/obs", 1,
    std::bind(&OnnxInferer::obsCallback, this, std::placeholders::_1));

  pub_output_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/onnx/output", 1);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&OnnxInferer::timerCallback, this));

  ready_ = true;
}

OnnxInferer::~OnnxInferer()
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

void OnnxInferer::timerCallback()
{
  if (!ready_) {return;}

  std::vector<float> obs_input;
  {
    std::lock_guard<std::mutex> lk(observation_mutex_);
    obs_input.assign(observation_.begin(), observation_.end());
  }


  std::array<int64_t, 2> input_shape{{1, static_cast<int64_t>(obs_dim_)}};

  Ort::Value input_tensor =
    Ort::Value::CreateTensor<float>(
    mem_, obs_input.data(), obs_input.size(),
    input_shape.data(), input_shape.size());

  auto outputs = sess_->Run(
    Ort::RunOptions{nullptr},
    const_cast<const char * const *>(in_names_.data()), &input_tensor, 1,
    const_cast<const char * const *>(out_names_.data()), 1);

  const float * outputs_float = outputs.front().GetTensorData<float>();

  std_msgs::msg::Float64MultiArray outputs_msg;
  outputs_msg.data.resize(outputs.front().GetTensorTypeAndShapeInfo().GetElementCount());
  for (size_t i = 0; i < outputs_msg.data.size(); ++i) {
    outputs_msg.data[i] = static_cast<double>(outputs_float[i]);
  }

  pub_output_->publish(outputs_msg);
}

void OnnxInferer::obsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != obs_dim_) {
    return;
  }
  std::lock_guard<std::mutex> lk(observation_mutex_);
  observation_.assign(msg->data.begin(), msg->data.end());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OnnxInferer>();
  if (!node->is_ready()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
