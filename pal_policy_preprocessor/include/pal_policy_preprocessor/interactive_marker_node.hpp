#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

class InteractiveTargetPublisher : public rclcpp::Node
{
public:
  InteractiveTargetPublisher();

private:
  // Publisher for the target position
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;

  // Interactive Marker server (constructed with this node's interfaces)
  interactive_markers::InteractiveMarkerServer server_;

  void makeInteractiveMarker();

  // Feedback callback from the IM server
  void processFeedback(
    const interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr & feedback);
};
