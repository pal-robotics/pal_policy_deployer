cmake_minimum_required(VERSION 3.8)
project(pal_policy_inferer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# ONNX Runtime
include_directories(
  include
  /opt/onnxruntime/include
  /opt/onnxruntime/include/onnxruntime/core/session
)
link_directories(/opt/onnxruntime/lib)

add_executable(${PROJECT_NAME}_ik_node src/inferer_tiago_ik_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_ik_node
  rclcpp
  std_msgs
  sensor_msgs
  trajectory_msgs
  ament_index_cpp
)

target_link_libraries(${PROJECT_NAME}_ik_node
  onnxruntime
)

install(TARGETS ${PROJECT_NAME}_ik_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
