#include "pal_policy_preprocessor/preprocessor.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <filesystem>
#include <functional>

#include <ament_index_cpp/get_package_share_directory.hpp>

namespace fs = std::filesystem;


// BASE CLASS IMPLEMENTATION
Preprocessor::Preprocessor(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options),
  env_(ORT_LOGGING_LEVEL_WARNING, "rl")
{
  pub_obs_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("preprocessor/obs", 1);
}

// MODEL PATH RESOLUTION >> should this go to deployer pkg?
std::string Preprocessor::resolveModelPath()
{
  model_path_ = this->declare_parameter<std::string>("onnx_model_path", "");

  if (!model_path_.empty()) {
    return model_path_;
  }

  const std::string pkg_share =
    ament_index_cpp::get_package_share_directory("pal_policy_deployer");

  const std::string config_dir =
    (fs::path(pkg_share) / "models" / "tiagopro" / "exported").string();

  if (fs::exists(config_dir) && fs::is_directory(config_dir)) {
    for (const auto & entry : fs::directory_iterator(config_dir)) {
      const auto & p = entry.path();
      if (entry.is_regular_file() && p.extension() == ".onnx") {
        model_path_ = p.string();
        RCLCPP_INFO(this->get_logger(), "Found ONNX model: %s", model_path_.c_str());
        return model_path_;
      }
    }
  }
  RCLCPP_FATAL(
    this->get_logger(),
    "onnx_model_path not set and no .onnx found in %s",
    config_dir.c_str());
  throw std::runtime_error("no onnx model found");
}

// BUILD METADATA MAP
void Preprocessor::buildMetadataMap(const std::string & onnx_model_path)
{
  if (onnx_model_path.empty()) {
    throw std::runtime_error("buildMetadataMap(): onnx_model_path is empty");
  }

  Ort::SessionOptions session_options;
  Ort::Session session(env_, onnx_model_path.c_str(), session_options);

  Ort::ModelMetadata metadata = session.GetModelMetadata();
  Ort::AllocatorWithDefaultOptions allocator;

  std::vector<Ort::AllocatedStringPtr> keys =
    metadata.GetCustomMetadataMapKeysAllocated(allocator);

  metadata_map_.clear();

  for (auto & key_ptr : keys) {
    const std::string key = key_ptr.get();

    Ort::AllocatedStringPtr value_ptr =
      metadata.LookupCustomMetadataMapAllocated(key.c_str(), allocator);

    if (value_ptr) {
      metadata_map_[key] = value_ptr.get();
    }
  }
}

// LOOP / RUN FUNCTION
void Preprocessor::run(std::chrono::milliseconds period)
{
  timer_ = this->create_wall_timer(
    period,
    std::bind(&Preprocessor::onTimer, this));
}

// HOOK TO CHILD CLASS
void Preprocessor::onTimer()
{
  std::vector<double> obs = buildObservationVector();
  publishObservation(obs);
}

void Preprocessor::publishObservation(const std::vector<double> & obs)
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data = obs;
  pub_obs_->publish(msg);
}

// -------------------- UTILITIES --------------------
std::vector<std::string>
Preprocessor::splitString(const std::string & str, char delimiter)
{
  std::vector<std::string> tokens;
  std::stringstream ss(str);
  std::string token;

  while (std::getline(ss, token, delimiter)) {
    if (!token.empty()) {
      tokens.push_back(token);
    }
  }
  return tokens;
}

std::vector<double>
Preprocessor::parseDoubles(const std::string & str)
{
  std::vector<double> values;
  std::stringstream ss(str);
  std::string token;

  while (std::getline(ss, token, ',')) {
    if (!token.empty()) {
      values.push_back(std::stod(token));
    }
  }
  return values;
}

bool Preprocessor::isExcludedJoint(
  const std::string & joint_name,
  const std::vector<std::string> & excluded)
{
  return std::find(excluded.begin(), excluded.end(), joint_name) != excluded.end();
}
