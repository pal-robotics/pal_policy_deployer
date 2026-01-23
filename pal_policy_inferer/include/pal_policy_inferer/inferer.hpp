#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <mutex>
#include <string>
#include <vector>

class Inferer : public rclcpp::Node
{
public:
  explicit Inferer(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~Inferer() override = default;

  bool is_ready() const {return ready_;}

protected:
  virtual void InferOutput(const std::vector<double> & obs, std::vector<double> & y_out) = 0;
  void set_ready(bool v) {ready_ = v;}
  size_t obs_dim_{0};
  std::vector<double> observation_;
  std::mutex observation_mutex_;
  std::vector<double> y_out_;

private:
  double start_delay_{0.5};
  void timerCallback();
  void obsCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg);

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_output_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_obs_;
  rclcpp::TimerBase::SharedPtr timer_;


  bool ready_{false};
};
