#include "pal_policy_preprocessor/interactive_marker_node.hpp"
#include <functional>

InteractiveTargetPublisher::InteractiveTargetPublisher()
: rclcpp::Node("pal_policy_preprocessor_interactive_node")
{
  pub_ = this->create_publisher<geometry_msgs::msg::Point>("/target_position", 10);
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
  int_marker.pose.position.x = 0.025;
  int_marker.pose.position.y = 0.141;
  int_marker.pose.position.z = -0.898;

  // 2) Control that allows 3D movement
  visualization_msgs::msg::InteractiveMarkerControl move_control;
  move_control.name = "move_3D";
  move_control.interaction_mode =
    visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;

  // 3) Visual: a small green sphere
  visualization_msgs::msg::Marker sphere;
  sphere.type = visualization_msgs::msg::Marker::SPHERE;
  sphere.scale.x = 0.1;
  sphere.scale.y = 0.1;
  sphere.scale.z = 0.1;
  sphere.color.r = 0.0f;
  sphere.color.g = 1.0f;
  sphere.color.b = 0.0f;
  sphere.color.a = 1.0f;

  move_control.markers.push_back(sphere);
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

  pub_->publish(p);

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
