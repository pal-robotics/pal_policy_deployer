#include "kang_mjlab_node.hpp"

#include <functional>

// Subscription/publisher template instantiations are kept apart from the rest of the node
// to limit the peak compiler memory per translation unit.

namespace pal_policy_inferer
{

void KangMjlabNode::createRosInterfaces()
{
  sub_js_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 1,
    std::bind(&KangMjlabNode::jsCB, this, std::placeholders::_1));
  sub_cmd_vel_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", 1,
    std::bind(&KangMjlabNode::velCmdCB, this, std::placeholders::_1));

  pub_sub_cmd_ = this->create_publisher<control_msgs::msg::MultiDOFCommand>(
    "/subscriber_controller/desired_state", 1);
}

void KangMjlabNode::createOdomSubscription()
{
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 1,
    std::bind(&KangMjlabNode::odomCB, this, std::placeholders::_1));
}

}  // namespace pal_policy_inferer
