#pragma once

#include "pal_policy_inferer/inferer.hpp"

#include <onnxruntime_cxx_api.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

class OnnxInferer : public Inferer
{
public:
  explicit OnnxInferer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OnnxInferer() override;

protected:
  std::vector<double> InferOutput(const std::vector<double> & obs) override;

private:
  std::string model_path_;

  // ONNX Runtime
  Ort::Env env_;
  Ort::MemoryInfo mem_;
  Ort::AllocatorWithDefaultOptions alloc_;
  std::unique_ptr<Ort::Session> sess_;

  char * in_name_{nullptr};
  char * out_name_{nullptr};

  int input_dim{0};
  int output_dim{0};
};
