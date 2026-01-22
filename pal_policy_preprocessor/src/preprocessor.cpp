#include "pal_policy_preprocessor/preprocessor.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <filesystem>
#include <functional>

namespace fs = std::filesystem;


// BASE CLASS IMPLEMENTATION
Preprocessor::Preprocessor(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options),
  env_(ORT_LOGGING_LEVEL_WARNING, "rl")
{
  pub_obs_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("/preprocessor/obs", 1);

  model_path_ = this->declare_parameter<std::string>("onnx_model_path", "");
  if (model_path_.empty()) {
    throw std::runtime_error("onnx_model_path param not set");
  }

}

// LOOP / RUN FUNCTION
void Preprocessor::run(std::chrono::milliseconds const period)
{
  timer_ = this->create_wall_timer(
    period,
    std::bind(&Preprocessor::onTimer, this));
}

// HOOK TO CHILD CLASS
void Preprocessor::onTimer()
{
  buildInputVector(&obs_);
  publishInput(obs_);
}

void Preprocessor::publishInput(const std::vector<double> & obs)
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
