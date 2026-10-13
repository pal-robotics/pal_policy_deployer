#include "kang_mjlab_node.hpp"

#include <functional>

// Kept apart from kang_mjlab_node_ros.cpp: all subscriptions in one translation unit need
// ~2 GB of compiler memory.

namespace pal_policy_inferer
{

void KangMjlabNode::createStatisticsSubscriptions()
{
  sub_stat_names_ = this->create_subscription<pal_statistics_msgs::msg::StatisticsNames>(
    "/controller_manager/introspection_data/names",
    rclcpp::QoS(1).reliable().transient_local(),
    std::bind(&KangMjlabNode::palStatisticsNamesCb, this, std::placeholders::_1));
  sub_stat_values_ = this->create_subscription<pal_statistics_msgs::msg::StatisticsValues>(
    "/controller_manager/introspection_data/values",
    rclcpp::QoS(1).best_effort(),
    std::bind(&KangMjlabNode::palStatisticsValuesCb, this, std::placeholders::_1));
}

}  // namespace pal_policy_inferer
