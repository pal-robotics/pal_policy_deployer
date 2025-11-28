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
  void leftPosCB(const geometry_msgs::msg::Point::SharedPtr msg);
  void leftQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg);
  void rightPosCB(const geometry_msgs::msg::Point::SharedPtr msg);
  void rightQuatCB(const geometry_msgs::msg::Quaternion::SharedPtr msg);

  void lastCB(const std_msgs::msg::Float64MultiArray::SharedPtr a);
  void onTimer();

  std::vector<double> obs_;

  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_left_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_left_quat_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_right_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_right_quat_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_last_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_obs_;
  rclcpp::TimerBase::SharedPtr timer_;
};
