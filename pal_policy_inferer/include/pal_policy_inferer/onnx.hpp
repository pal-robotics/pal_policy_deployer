#pragma once

#include "pal_policy_inferer/inferer.hpp"

#include <onnxruntime_cxx_api.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace pal_policy_inferer
{

class OnnxInferer : public Inferer
{
public:
  explicit OnnxInferer();
  ~OnnxInferer() override;

protected:
  void InferOutput(const std::vector<double> & obs, std::vector<double> * y) override;

private:
  std::string model_path_;

  std::vector<float> obs_f_;
  std::array<int64_t, 2> input_shape_{};

  Ort::Env env_;
  Ort::MemoryInfo mem_;
  Ort::AllocatorWithDefaultOptions alloc_;
  std::unique_ptr<Ort::Session> sess_;

  char * in_name_{nullptr};
  char * out_name_{nullptr};
};

} // pal_policy_inferer
