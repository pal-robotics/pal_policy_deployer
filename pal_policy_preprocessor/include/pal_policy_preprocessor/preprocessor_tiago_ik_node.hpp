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

  // name -> index map for JointState
  std::unordered_map<std::string, size_t> joint_index_;
  std::vector<std::string> last_js_names_;
  std::vector<std::string> expected_joint_names_;
  bool joint_map_ready_{false};


  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_left_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_left_quat_;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_right_pos_;
  rclcpp::Subscription<geometry_msgs::msg::Quaternion>::SharedPtr sub_right_quat_;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_last_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_js_;

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_obs_;
  rclcpp::TimerBase::SharedPtr timer_;
};
