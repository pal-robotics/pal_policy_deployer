cmake_minimum_required(VERSION 3.8)
project(pal_policy_inferer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(_Boost_PYTHON310_HEADERS "boost/python.hpp")
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ONNX Runtime
include_directories(SYSTEM include 
  /opt/onnxruntime/include
)
link_directories(/opt/onnxruntime/lib)

add_executable(${PROJECT_NAME}_ik_node src/inferer_tiago_ik_node.cpp)

ament_target_dependencies(${PROJECT_NAME}_ik_node
  rclcpp
  std_msgs
  ament_index_cpp
  geometry_msgs          
  tf2                     
  tf2_ros                 
  tf2_geometry_msgs
)

target_link_libraries(${PROJECT_NAME}_ik_node
  onnxruntime
)

install(TARGETS ${PROJECT_NAME}_ik_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
