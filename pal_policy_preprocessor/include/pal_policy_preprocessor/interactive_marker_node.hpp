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
  void init_server();

private:
  // Publishes the target position
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr pub_;

  // Interactive Marker server
  std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;

  void makeInteractiveMarker();

  // Feedback callback from the IM server
  void processFeedback(
    const interactive_markers::InteractiveMarkerServer::FeedbackConstSharedPtr & feedback);
};
