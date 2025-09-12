// src/interactive_target_publisher.cpp

#include <ros/ros.h>
#include <geometry_msgs/Point.h>

#include <interactive_markers/interactive_marker_server.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/Marker.h>

using namespace visualization_msgs;
using namespace interactive_markers;

class InteractiveTargetPublisher
{
public:
  InteractiveTargetPublisher()
: server_("target_marker")
  {
    // Publisher for the target position
    pub_ = nh_.advertise<geometry_msgs::Point>("/target_position", 10);

    // Create and insert our interactive marker
    makeInteractiveMarker();
    server_.applyChanges();
  }

private:
  ros::NodeHandle             nh_;
  ros::Publisher              pub_;
  InteractiveMarkerServer     server_;

  void makeInteractiveMarker()
  {
    // 1) Define the basic InteractiveMarker
    InteractiveMarker int_marker;
    int_marker.header.frame_id = "base_link";
    int_marker.name            = "target";
    int_marker.description     = "Drag to move target";
    int_marker.scale           = 0.2;
    int_marker.pose.position.x = 0.025;
    int_marker.pose.position.y = 0.141;
    int_marker.pose.position.z = -0.898;
    
    // 2) Create a control that allows 3D movement
    InteractiveMarkerControl move_control;
    move_control.name = "move_3D";
    move_control.interaction_mode = InteractiveMarkerControl::MOVE_3D;

    // 3) Visual: a small green sphere
    Marker sphere;
    sphere.type = Marker::SPHERE;
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
    server_.insert(int_marker,
                   boost::bind(&InteractiveTargetPublisher::processFeedback, this, _1));
  }

  void processFeedback(const InteractiveMarkerFeedbackConstPtr &feedback)
  {
    geometry_msgs::Point p;
    p.x = feedback->pose.position.x;
    p.y = feedback->pose.position.y;
    p.z = feedback->pose.position.z;

    pub_.publish(p);
    ROS_INFO_STREAM("Published new target: ["
                    << p.x << ", "
                    << p.y << ", "
                    << p.z << "]");
    server_.applyChanges();  // update marker server
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "interactive_target_publisher");
  InteractiveTargetPublisher node;
  ros::spin();
  return 0;
}
