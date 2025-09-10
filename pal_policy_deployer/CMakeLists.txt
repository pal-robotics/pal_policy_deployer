cmake_minimum_required(VERSION 3.0.2)
project(pal_policy_deployer)

find_package(Boost REQUIRED COMPONENTS filesystem system)

add_message_files(
  FILES
  StampedInt8Array.msg
  StampedFloat64Array.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime message_filters
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_generation
  message_filters
  visualization_msgs
  interactive_markers
  pal_policy_inferer
  pal_policy_postprocessor
  pal_policy_preprocessor
)




include_directories(
  ${catkin_INCLUDE_DIRS}
)

