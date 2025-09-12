cmake_minimum_required(VERSION 3.0.2)
project(pal_policy_postprocessor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_filters
  visualization_msgs
  interactive_markers
)

catkin_package(
  CATKIN_DEPENDS
    roscpp std_msgs message_filters visualization_msgs interactive_markers
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)