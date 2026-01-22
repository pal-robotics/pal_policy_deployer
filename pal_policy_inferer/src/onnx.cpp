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

  // create session
  Ort::SessionOptions so;
  sess_.reset(new Ort::Session(env_, model_path_.c_str(), so));

  {
    std::lock_guard<std::mutex> lk(observation_mutex_);
    observation_.assign(input_dim, 0.0);
  }
  in_name_ = sess_->GetInputNameAllocated(0, alloc_).release();
  out_name_ = sess_->GetOutputNameAllocated(0, alloc_).release();

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

std::vector<double> OnnxInferer::InferOutput(const std::vector<double> & obs)
{
  std::vector<float> obs_f(obs.begin(), obs.end());

  std::array<int64_t, 2> input_shape{{1, static_cast<int64_t>(obs_f.size())}};

  Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
    mem_,
    obs_f.data(),
    obs_f.size(),
    input_shape.data(),
    input_shape.size());

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
  std::vector<double> y(output_dim);
  for (size_t i = 0; i < output_dim; ++i) {
    y[i] = static_cast<double>(y_f[i]);
  }
  return y;
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
