#include "pal_policy_postprocessor/postprocessor.hpp"

#include <algorithm>
#include <sstream>
#include <stdexcept>
#include <filesystem>
#include <functional>

namespace fs = std::filesystem;

namespace pal_policy_postprocessor
{

// BASE CLASS IMPLEMENTATION
Postprocessor::Postprocessor(const std::string & node_name)
: rclcpp::Node(node_name)
{
  sub_output_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/inferer/output", 1,
    std::bind(&Postprocessor::outputCallback, this, std::placeholders::_1));
}

void Postprocessor::outputCallback(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  postprocessAndPublish(msg->data); // hook to child class
}

// -------------------- UTILITIES --------------------
std::vector<double>
Postprocessor::parseDoubles(const std::string & str)
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
}
