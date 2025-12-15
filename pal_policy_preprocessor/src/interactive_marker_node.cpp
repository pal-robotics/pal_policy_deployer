#include "pal_policy_preprocessor/interactive_marker_node.hpp"
#include <functional>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

InteractiveTargetPublisher::InteractiveTargetPublisher()
: rclcpp::Node("pal_policy_preprocessor_interactive_node")
{
  // Left target publishers
  pub_left_pos_ = this->create_publisher<geometry_msgs::msg::Point>(
    "/left_target_position", 10);
  pub_left_quat_ = this->create_publisher<geometry_msgs::msg::Quaternion>(
    "/left_target_orientation", 10);

  // Right target publishers
  pub_right_pos_ = this->create_publisher<geometry_msgs::msg::Point>(
    "/right_target_position", 10);
  pub_right_quat_ = this->create_publisher<geometry_msgs::msg::Quaternion>(
    "/right_target_orientation", 10);
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
  // Common control (3D move + rotate, colored axes)
  visualization_msgs::msg::InteractiveMarkerControl move_control;
  move_control.name = "move_rotate_3D";
  move_control.interaction_mode =
    visualization_msgs::msg::InteractiveMarkerControl::MOVE_ROTATE_3D;
  move_control.always_visible = true;

  // We will configure arrows based on the first marker's scale
  double arrow_length = 0.2 * 0.8;   // scale * 0.8
  double arrow_radius = 0.2 * 0.05;  // scale * 0.05

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

  // Y-axis arrow (green)
  visualization_msgs::msg::Marker y_arrow = x_arrow;
  y_arrow.color.r = 0.0f;
  y_arrow.color.g = 1.0f;
  y_arrow.pose.orientation.w = 0.7071;
  y_arrow.pose.orientation.z = 0.7071;  // rotate 90° around Z

  // Z-axis arrow (blue)
  visualization_msgs::msg::Marker z_arrow = x_arrow;
  z_arrow.color.r = 0.0f;
  z_arrow.color.g = 0.0f;
  z_arrow.color.b = 1.0f;
  z_arrow.pose.orientation.w = 0.7071;
  z_arrow.pose.orientation.y = -0.7071; // rotate 90° around Y

  // ---------------- LEFT MARKER ----------------
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_footprint";
    int_marker.name = "left_target";
    int_marker.description = "Left target";
    int_marker.scale = 0.2;

    // Initial pose (you can tune these)
    int_marker.pose.position.x = 0.272;
    int_marker.pose.position.y = 0.209;   // mirror sign from original
    int_marker.pose.position.z = 0.560;

    int_marker.pose.orientation.x = 0.21066;
    int_marker.pose.orientation.y = 0.0041078;
    int_marker.pose.orientation.z = 0.97441;
    int_marker.pose.orientation.w = -0.078344;

    visualization_msgs::msg::InteractiveMarkerControl left_control = move_control;
    left_control.markers.clear();
    left_control.markers.push_back(x_arrow);
    left_control.markers.push_back(y_arrow);
    left_control.markers.push_back(z_arrow);

    int_marker.controls.push_back(left_control);

    server_->insert(
      int_marker,
      std::bind(&InteractiveTargetPublisher::processFeedback, this, std::placeholders::_1));
  }

  // ---------------- RIGHT MARKER ----------------
  {
    visualization_msgs::msg::InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_footprint";
    int_marker.name = "right_target";
    int_marker.description = "Right target";
    int_marker.scale = 0.2;

    // Initial pose (original single marker values)
    int_marker.pose.position.x = 0.272;
    int_marker.pose.position.y = -0.209;
    int_marker.pose.position.z = 0.560;

    int_marker.pose.orientation.x = 0.21066;
    int_marker.pose.orientation.y = 0.0041078;
    int_marker.pose.orientation.z = 0.97441;
    int_marker.pose.orientation.w = -0.078344;

    visualization_msgs::msg::InteractiveMarkerControl right_control = move_control;
    right_control.markers.clear();
    right_control.markers.push_back(x_arrow);
    right_control.markers.push_back(y_arrow);
    right_control.markers.push_back(z_arrow);

    int_marker.controls.push_back(right_control);

    server_->insert(
      int_marker,
      std::bind(&InteractiveTargetPublisher::processFeedback, this, std::placeholders::_1));
  }
}

void InteractiveTargetPublisher::processFeedback(
  const interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr & feedback)
{
  // Normalize quaternion
  tf2::Quaternion tq;
  tf2::fromMsg(feedback->pose.orientation, tq);
  tq.normalize();
  geometry_msgs::msg::Quaternion q = tf2::toMsg(tq);

  geometry_msgs::msg::Point p;
  p.x = feedback->pose.position.x;
  p.y = feedback->pose.position.y;
  p.z = feedback->pose.position.z;

  // Decide which marker this feedback belongs to
  if (feedback->marker_name == "left_target") {
    pub_left_pos_->publish(p);
    pub_left_quat_->publish(q);
  } else if (feedback->marker_name == "right_target") {
    pub_right_pos_->publish(p);
    pub_right_quat_->publish(q);
  }

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
