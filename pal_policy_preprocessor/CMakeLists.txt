cmake_minimum_required(VERSION 3.8)
project(pal_policy_preprocessor)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_library(ONNXRUNTIME_LIB onnxruntime REQUIRED)
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATHS /usr/local/include/onnxruntime
  REQUIRED)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/preprocessor.cpp
  src/preprocessor_tiago_reaching.cpp
)

target_include_directories(${PROJECT_NAME}_node PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_node ${ONNXRUNTIME_LIB})

ament_auto_add_executable(interactive_marker_node
  src/interactive_marker_node.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  config
)

