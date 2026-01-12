cmake_minimum_required(VERSION 3.8)
project(pal_policy_preprocessor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(ament_index_cpp REQUIRED)

find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# ---- ONNX Runtime paths (your current layout) ----
set(ONNXRUNTIME_ROOT "/opt/onnxruntime" CACHE PATH "ONNX Runtime install prefix")

find_path(ONNXRUNTIME_INCLUDE_DIR
  NAMES onnxruntime_cxx_api.h
  HINTS ${ONNXRUNTIME_ROOT}/include
  NO_DEFAULT_PATH
)

# Prefer the full library file so we don't rely on link_directories()
find_library(ONNXRUNTIME_LIB
  NAMES onnxruntime
  HINTS ${ONNXRUNTIME_ROOT}/lib ${ONNXRUNTIME_ROOT}/lib64
  NO_DEFAULT_PATH
)

if(NOT ONNXRUNTIME_INCLUDE_DIR)
  message(FATAL_ERROR "Could not find onnxruntime headers. Expected ${ONNXRUNTIME_ROOT}/include/onnxruntime_cxx_api.h")
endif()

if(NOT ONNXRUNTIME_LIB)
  message(FATAL_ERROR "Could not find onnxruntime library. Expected ${ONNXRUNTIME_ROOT}/lib/libonnxruntime.so (or lib64)")
endif()

# -------------------- IK NODE (uses onnxruntime) --------------------
add_executable(${PROJECT_NAME}_ik_node
  src/preprocessor_tiago_ik_node.cpp
)

target_include_directories(${PROJECT_NAME}_ik_node SYSTEM PRIVATE
  include
  ${ONNXRUNTIME_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}_ik_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  ament_index_cpp
)

target_link_libraries(${PROJECT_NAME}_ik_node
  ${ONNXRUNTIME_LIB}
)

# -------------------- INTERACTIVE NODE (no onnxruntime) --------------------
add_executable(${PROJECT_NAME}_interactive_node
  src/interactive_marker_node.cpp
)

target_include_directories(${PROJECT_NAME}_interactive_node SYSTEM PRIVATE
  include
)

ament_target_dependencies(${PROJECT_NAME}_interactive_node
  rclcpp
  std_msgs
  ament_index_cpp
  visualization_msgs
  interactive_markers
  geometry_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS
  ${PROJECT_NAME}_ik_node
  ${PROJECT_NAME}_interactive_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
