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
  void InferOutput(const std::vector<double> & obs, std::vector<double> & y_out) override;

private:
  std::string model_path_;

  // ONNX Runtime
  Ort::Env env_;
  Ort::MemoryInfo mem_;
  Ort::AllocatorWithDefaultOptions alloc_;
  std::unique_ptr<Ort::Session> sess_;

  char * in_name_{nullptr};
  char * out_name_{nullptr};

  const char * input_names_[1]{nullptr};
  const char * output_names_[1]{nullptr};

  int64_t input_dim{0};
  int64_t output_dim{0};

  std::vector<float> input_f_;
  std::array<int64_t, 2> input_shape_{{1, 0}};

};
