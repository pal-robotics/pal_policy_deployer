#include "pal_policy_inferer/kang_mjlab_inference.hpp"

#include <onnxruntime_cxx_api.h>

#include <filesystem>
#include <map>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace pal_policy_inferer
{

namespace
{
// One environment per process is enough; ORT allows several sessions on one Env.
Ort::Env & ortEnv()
{
  static Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "rl");
  return env;
}

std::string shapeToString(const std::vector<int64_t> & dims)
{
  std::stringstream ss;
  for (size_t j = 0; j < dims.size(); j++) {
    if (j > 0) {ss << ",";}
    ss << dims[j];
  }
  return ss.str();
}
}  // namespace

struct KangMjlabModel::Impl
{
  Ort::MemoryInfo mem{Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault)};
  std::unique_ptr<Ort::Session> sess;
  // Owning name strings; in_names/out_names are the raw views Run() expects.
  std::vector<Ort::AllocatedStringPtr> name_storage;
  std::vector<const char *> in_names;
  std::vector<const char *> out_names;
  std::vector<int64_t> input_shape;
  std::vector<int64_t> time_step_shape{1, 1};
  int out_idx_actions = -1;
  int out_idx_joint_pos = -1;
  int out_idx_joint_vel = -1;
  std::vector<Ort::Value> outputs;  // kept alive so Outputs pointers stay valid
};

KangMjlabModel::KangMjlabModel()
: impl_(std::make_unique<Impl>()) {}
KangMjlabModel::~KangMjlabModel() = default;
KangMjlabModel::KangMjlabModel(KangMjlabModel &&) noexcept = default;
KangMjlabModel & KangMjlabModel::operator=(KangMjlabModel &&) noexcept = default;

std::string KangMjlabModel::findOnnxModel(const std::string & dir)
{
  if (!fs::is_directory(dir)) {
    throw std::runtime_error("Model directory does not exist: " + dir);
  }
  for (const auto & entry : fs::directory_iterator(dir)) {
    const fs::path & fp = entry.path();
    if (fs::is_regular_file(fp) && fp.extension() == ".onnx") {
      return fp.string();
    }
  }
  throw std::runtime_error("No ONNX model found in " + dir);
}

std::vector<std::string> KangMjlabModel::load(
  const std::string & model_path, PolicyConfig & config, const MotionState & motion)
{
  auto & s = *impl_;
  s.sess = std::make_unique<Ort::Session>(ortEnv(), model_path.c_str(), Ort::SessionOptions{});
  Ort::Session & sess = *s.sess;
  Ort::AllocatorWithDefaultOptions allocator;

  Ort::ModelMetadata metadata = sess.GetModelMetadata();
  std::map<std::string, std::string> metadata_map;
  for (auto & key_ptr : metadata.GetCustomMetadataMapKeysAllocated(allocator)) {
    std::string key = key_ptr.get();
    Ort::AllocatedStringPtr value_ptr =
      metadata.LookupCustomMetadataMapAllocated(key.c_str(), allocator);
    if (value_ptr) {
      metadata_map[key] = value_ptr.get();
    }
  }
  parsePolicyMetadata(metadata_map, config);

  config.num_inputs = static_cast<int>(sess.GetInputCount());
  config.num_outputs = static_cast<int>(sess.GetOutputCount());
  config.obs_dim =
    static_cast<int>(sess.GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape()[1]);

  std::vector<std::string> io;
  s.name_storage.clear();
  s.in_names.clear();
  s.out_names.clear();
  for (int i = 0; i < config.num_outputs; ++i) {
    s.name_storage.push_back(sess.GetOutputNameAllocated(i, allocator));
    const std::string n = s.name_storage.back().get();
    s.out_names.push_back(s.name_storage.back().get());
    const auto dims = sess.GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    io.push_back("Output " + std::to_string(i) + ": '" + n + "' [" + shapeToString(dims) + "]");

    if (n == "actions") {
      config.out_idx_actions = i;
      config.action_dim = static_cast<int>(dims[1]);
    } else if (n == "joint_pos") {
      config.out_idx_joint_pos = i;
    } else if (n == "joint_vel") {
      config.out_idx_joint_vel = i;
    }
  }
  for (int i = 0; i < config.num_inputs; ++i) {
    s.name_storage.push_back(sess.GetInputNameAllocated(i, allocator));
    s.in_names.push_back(s.name_storage.back().get());
    const auto dims = sess.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
    io.push_back(
      "Input " + std::to_string(i) + ": '" + s.in_names.back() + "' [" +
      shapeToString(dims) + "]");
  }

  // For imitation, single_obs_dim is deferred until motion.num_joints is known.
  if (config.task != "imitation" && !computeHistoryLength(config, motion)) {
    throw std::runtime_error(
            "Observation dimension " + std::to_string(config.obs_dim) +
            " not divisible by single obs dim " + std::to_string(config.single_obs_dim));
  }

  s.input_shape = {1, static_cast<int64_t>(config.obs_dim)};
  s.out_idx_actions = config.out_idx_actions;
  s.out_idx_joint_pos = config.out_idx_joint_pos;
  s.out_idx_joint_vel = config.out_idx_joint_vel;
  return io;
}

KangMjlabModel::Outputs KangMjlabModel::run(
  std::vector<float> & obs, float time_step, bool with_time_step)
{
  auto & s = *impl_;
  std::vector<Ort::Value> inputs;
  inputs.push_back(
    Ort::Value::CreateTensor<float>(
      s.mem, obs.data(), obs.size(), s.input_shape.data(), s.input_shape.size()));
  if (with_time_step) {
    inputs.push_back(
      Ort::Value::CreateTensor<float>(
        s.mem, &time_step, 1, s.time_step_shape.data(), s.time_step_shape.size()));
  }

  s.outputs = s.sess->Run(
    Ort::RunOptions{nullptr},
    s.in_names.data(), inputs.data(), inputs.size(),
    s.out_names.data(), s.out_names.size());

  auto data = [&](int idx) -> const float * {
      return idx >= 0 ? s.outputs[idx].GetTensorData<float>() : nullptr;
    };
  return {data(s.out_idx_actions), data(s.out_idx_joint_pos), data(s.out_idx_joint_vel)};
}

}  // namespace pal_policy_inferer
