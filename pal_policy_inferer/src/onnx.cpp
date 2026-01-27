#include "pal_policy_inferer/onnx.hpp"

#include <array>
#include <mutex>
#include <stdexcept>
#include <vector>

namespace pal_policy_inferer
{

OnnxInferer::OnnxInferer(const rclcpp::NodeOptions & options)
: Inferer("onnx", options),
  env_(ORT_LOGGING_LEVEL_WARNING, "rl"),
  mem_(Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault))
{
  this->declare_parameter<std::string>("onnx_model_path", "");
  this->get_parameter("onnx_model_path", model_path_);
  if (model_path_.empty()) {
    throw std::runtime_error("onnx_model_path param not set");
  }

  try {
    Ort::SessionOptions so;
    sess_.reset(new Ort::Session(env_, model_path_.c_str(), so));

    // --- input dims ---
    Ort::TypeInfo input_info = sess_->GetInputTypeInfo(0);
    auto in_tensor_info = input_info.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> in_dims = in_tensor_info.GetShape();

    if (in_dims.size() < 2 || in_dims[1] <= 0) {
      throw std::runtime_error(
              "ONNX obs dim is dynamic/invalid (expected fixed [1, N])");
    }
    obs_dim_ = static_cast<size_t>(in_dims[1]);

    obs_f_.assign(obs_dim_, 0.0f);
    obs_local.assign(obs_dim_, 0.0);
    input_shape_ = {{1, static_cast<int64_t>(obs_dim_)}};

    // --- output dims ---
    Ort::TypeInfo output_info = sess_->GetOutputTypeInfo(0);
    auto out_tensor_info = output_info.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> out_dims = out_tensor_info.GetShape();

    if (out_dims.size() < 2 || out_dims[1] <= 0) {
      throw std::runtime_error(
              "ONNX output dim is dynamic/invalid (expected fixed [1, M])");
    }
    output_dim_ = static_cast<size_t>(out_dims[1]);

    y_out_.assign(output_dim_, 0.0);

    {
      std::lock_guard<std::mutex> lk(observation_mutex_);
      observation_.assign(obs_dim_, 0.0);
    }

    in_name_ = sess_->GetInputNameAllocated(0, alloc_).release();
    out_name_ = sess_->GetOutputNameAllocated(0, alloc_).release();

  } catch (const Ort::Exception & e) {
    RCLCPP_FATAL(
      this->get_logger(),
      "Failed to initialize ONNX inferer for model '%s': %s",
      model_path_.c_str(), e.what());
    throw;
  }

  set_ready(true);
}

OnnxInferer::~OnnxInferer()
{
  if (in_name_) {
    alloc_.Free(in_name_);
    in_name_ = nullptr;
  }
  if (out_name_) {
    alloc_.Free(out_name_);
    out_name_ = nullptr;
  }
}

void OnnxInferer::InferOutput(const std::vector<double> & obs, std::vector<double> * y)
{
  if (!y) {
    throw std::runtime_error("InferOutput: y is null");
  }
  if (obs.size() != obs_dim_) {
    throw std::runtime_error("InferOutput: obs size mismatch");
  }

  for (size_t i = 0; i < obs_dim_; ++i) {
    obs_f_[i] = static_cast<float>(obs[i]);
  }

  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
    mem_,
    obs_f_.data(),
    obs_f_.size(),
    input_shape_.data(),
    input_shape_.size());

  const char * input_names[] = {in_name_};
  const char * output_names[] = {out_name_};

  auto outputs = sess_->Run(
    Ort::RunOptions{nullptr},
    input_names,
    &input_tensor,
    1,
    output_names,
    1);

  const float * y_f = outputs.front().GetTensorData<float>();

  for (size_t i = 0; i < output_dim_; ++i) {
    (*y)[i] = static_cast<double>(y_f[i]);
  }
}

} // namespace pal_policy_inferer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<pal_policy_inferer::OnnxInferer>(options);
  if (!node->is_ready()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
