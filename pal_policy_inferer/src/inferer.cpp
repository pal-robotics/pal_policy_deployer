#include "pal_policy_inferer/inferer.hpp"

#include <chrono>
#include <functional>

// BASE CLASS IMPLEMENTATION
Inferer::Inferer(const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
{
  sub_obs_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    "/preprocessor/obs", 1,
    std::bind(&Inferer::obsCallback, this, std::placeholders::_1));

  pub_output_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    "/inferer/output", 1);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&Inferer::timerCallback, this));

  this->declare_parameter<double>("start_delay", 0.5);
  this->get_parameter("start_delay", start_delay_);
  RCLCPP_INFO(this->get_logger(), "Waiting %.1f seconds before starting...", start_delay_);
  rclcpp::sleep_for(std::chrono::milliseconds(static_cast<int64_t>(start_delay_ * 1000)));
}

// HOOK TO CHILD CLASS
void Inferer::timerCallback()
{
  if (!ready_) {
    return;
  }

  {
    std::lock_guard<std::mutex> lk(observation_mutex_);
    InferOutput(observation_, y_out_);
  }


  std_msgs::msg::Float64MultiArray outputs_msg;
  outputs_msg.data = std::move(y_out_);
  pub_output_->publish(outputs_msg);
}

void Inferer::obsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (msg->data.size() != obs_dim_) {
    return;
  }

  std::lock_guard<std::mutex> lk(observation_mutex_);
  observation_.assign(msg->data.begin(), msg->data.end());
}
