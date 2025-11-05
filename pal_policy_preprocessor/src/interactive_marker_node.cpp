#include "pal_policy_preprocessor/interactive_marker_node.hpp"
#include <functional>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

InteractiveTargetPublisher::InteractiveTargetPublisher()
: rclcpp::Node("pal_policy_preprocessor_interactive_node")
{
  pub_pos_ = this->create_publisher<geometry_msgs::msg::Point>("/target_position", 10);
  pub_quat_ = this->create_publisher<geometry_msgs::msg::Quaternion>("/target_orientation", 10);
}

void InteractiveTargetPublisher::init_server()
{
  auto node_ptr = this->shared_from_this();

  server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>(
    "target_marker", node_ptr);

  makeInteractiveMarker();
  server_->applyChanges();
}

void InteractiveTargetPublisher::makeInteractiveMarker()
{
  // 1) Define the InteractiveMarker
  visualization_msgs::msg::InteractiveMarker int_marker;
  int_marker.header.frame_id = "base_footprint";
  int_marker.name = "target";
  int_marker.description = "Drag to move target";
  int_marker.scale = 0.2;
  int_marker.pose.position.x = 0.272;
  int_marker.pose.position.y = -0.209;
  int_marker.pose.position.z = 0.560;

  int_marker.pose.orientation.x = 0.188;
  int_marker.pose.orientation.y = 0.004;
  int_marker.pose.orientation.z = 0.979;
  int_marker.pose.orientation.w = -0.078;

  // 2) Control that allows 3D movement
  visualization_msgs::msg::InteractiveMarkerControl move_control;
  move_control.name = "move_rotate_3D";
  move_control.interaction_mode =
    visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;

  double arrow_length = int_marker.scale * 0.8;
  double arrow_radius = int_marker.scale * 0.05;

  // X-axis arrow (red)
  visualization_msgs::msg::Marker x_arrow;
  x_arrow.type = visualization_msgs::msg::Marker::ARROW;
  x_arrow.scale.x = arrow_length;
  x_arrow.scale.y = arrow_radius;
  x_arrow.scale.z = arrow_radius;
  x_arrow.color.r = 1.0f;
  x_arrow.color.g = 0.0f;
  x_arrow.color.b = 0.0f;
  x_arrow.color.a = 1.0f;
  x_arrow.pose.orientation.w = 1.0;
  move_control.markers.push_back(x_arrow);

  // Y-axis arrow (green)
  visualization_msgs::msg::Marker y_arrow = x_arrow;
  y_arrow.color.r = 0.0f;
  y_arrow.color.g = 1.0f;
  y_arrow.pose.orientation.w = 0.7071;
  y_arrow.pose.orientation.z = 0.7071;  // rotate 90° around Z
  move_control.markers.push_back(y_arrow);

  // Z-axis arrow (blue)
  visualization_msgs::msg::Marker z_arrow = x_arrow;
  z_arrow.color.r = 0.0f;
  z_arrow.color.g = 0.0f;
  z_arrow.color.b = 1.0f;
  z_arrow.pose.orientation.w = 0.7071;
  z_arrow.pose.orientation.y = -0.7071; // rotate 90° around Y
  move_control.markers.push_back(z_arrow);

  move_control.always_visible = true;
  int_marker.controls.push_back(move_control);

  // 4) Insert and set feedback callback
  server_->insert(
    int_marker,
    std::bind(&InteractiveTargetPublisher::processFeedback, this, std::placeholders::_1));
}

void InteractiveTargetPublisher::processFeedback(
  const interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr & feedback)
{
  geometry_msgs::msg::Point p;
  p.x = feedback->pose.position.x;
  p.y = feedback->pose.position.y;
  p.z = feedback->pose.position.z;

  pub_pos_->publish(p);

  tf2::Quaternion tq;
  tf2::fromMsg(feedback->pose.orientation, tq);
  tq.normalize();
  geometry_msgs::msg::Quaternion q = tf2::toMsg(tq);
  pub_quat_->publish(q);

  server_->applyChanges();  // update marker server
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InteractiveTargetPublisher>();
  node->init_server();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
