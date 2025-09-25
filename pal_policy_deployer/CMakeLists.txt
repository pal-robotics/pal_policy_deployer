cmake_minimum_required(VERSION 3.8)
project(pal_policy_deployer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(_Boost_PYTHON310_HEADERS "boost/python.hpp")
find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include SYSTEM) 

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StampedInt8Array.msg"
  "msg/StampedFloat64Array.msg"
  DEPENDENCIES std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_auto_package(
  INSTALL_TO_SHARE
  models
  launch
)

