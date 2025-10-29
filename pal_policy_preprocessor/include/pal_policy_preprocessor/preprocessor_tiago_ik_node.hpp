#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

class IKObsPublisher : public rclcpp::Node
{
public:
  IKObsPublisher();
  ~IKObsPublisher() override = default;

private:
  void jsCB(const sensor_msgs::msg::JointState::SharedPtr msg);
  void poseCB(const geometry_msgs::msg::Point::SharedPtr msg);
  void lastCB(const std_msgs::msg::Float64MultiArray::SharedPtr a);
  void quatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg);
  void onTimer();

  std::vector<double> obs_;  // size 24: 7 pos, 7 vel, 7 delta, 3 target

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_pose_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_last_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_obs_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_quat_;
  rclcpp::TimerBase::SharedPtr timer_;
};
