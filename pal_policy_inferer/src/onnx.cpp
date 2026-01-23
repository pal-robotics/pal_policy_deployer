#include "pal_policy_inferer/onnx.hpp"

#include <stdexcept>

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

  this->declare_parameter<int64_t>("input_dimension", 0);
  this->declare_parameter<int64_t>("output_dimension", 0);

  input_dim = this->get_parameter("input_dimension").as_int();
  output_dim = this->get_parameter("output_dimension").as_int();

  input_f_.resize(static_cast<size_t>(input_dim));
  input_shape_ = {{1, input_dim}};

  // create session
  Ort::SessionOptions so;
  sess_.reset(new Ort::Session(env_, model_path_.c_str(), so));

  {
    std::lock_guard<std::mutex> lk(observation_mutex_);
    observation_.assign(input_dim, 0.0);
  }
  in_name_ = sess_->GetInputNameAllocated(0, alloc_).release();
  out_name_ = sess_->GetOutputNameAllocated(0, alloc_).release();

  input_names_[0] = in_name_;
  output_names_[0] = out_name_;

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

void OnnxInferer::InferOutput(const std::vector<double> & obs, std::vector<double> & y_out)
{
  y_out.assign(static_cast<size_t>(output_dim), 0.0);

  for (int64_t i = 0; i < input_dim; ++i) {
    input_f_[static_cast<size_t>(i)] = static_cast<float>(obs[static_cast<size_t>(i)]);
  }

  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
    mem_,
    input_f_.data(),
    input_f_.size(),
    input_shape_.data(),
    input_shape_.size());

  auto outputs = sess_->Run(
    Ort::RunOptions{nullptr},
    input_names_,
    &input_tensor,
    1,
    output_names_,
    1);

  const float * y_f = outputs.front().GetTensorData<float>();
  for (int64_t i = 0; i < output_dim; ++i) {
    y_out[static_cast<size_t>(i)] = static_cast<double>(y_f[static_cast<size_t>(i)]);
  }
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
