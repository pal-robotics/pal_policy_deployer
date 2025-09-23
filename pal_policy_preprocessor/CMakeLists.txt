cmake_minimum_required(VERSION 3.8)
project(pal_policy_preprocessor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(_Boost_PYTHON310_HEADERS "boost/python.hpp")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)


# ONNX Runtime
include_directories(SYSTEM include 
)

add_executable(${PROJECT_NAME}_ik_node src/preprocessor_tiago_ik_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_ik_node
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  ament_index_cpp
)

target_link_libraries(${PROJECT_NAME}_ik_node
)

install(TARGETS ${PROJECT_NAME}_ik_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
