#include "pal_policy_preprocessor/preprocessor.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <filesystem>
#include <functional>

namespace fs = std::filesystem;

namespace pal_policy_preprocessor
{

// BASE CLASS IMPLEMENTATION
Preprocessor::Preprocessor(const std::string & node_name)
: rclcpp::Node(node_name)
{
  pub_obs_ =
    this->create_publisher<std_msgs::msg::Float64MultiArray>("/preprocessor/obs", 1);

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

} // pal_policy_preprocessor
