#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

class InteractiveTargetPublisher : public rclcpp::Node
{
public:
  InteractiveTargetPublisher();
  void init_server();

private:
  // Publishes the target positions and orientations
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_left_pos_;
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr pub_left_quat_;

  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_right_pos_;
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr pub_right_quat_;

  // Interactive Marker server
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  // Create both left and right markers
  void makeInteractiveMarker();

  // Feedback callback from the IM server (handles both markers)
  void processFeedback(
    const interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr & feedback);
};
