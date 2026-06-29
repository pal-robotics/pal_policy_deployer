cmake_minimum_required(VERSION 3.8)
project(pal_policy_preprocessor)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node
  src/preprocessor.cpp
  src/preprocessor_tiago_reaching.cpp
)
target_link_libraries(${PROJECT_NAME}_node onnxruntime::onnxruntime)

ament_auto_add_executable(interactive_marker_node
  src/interactive_marker_node.cpp
)

if(ament_cmake_auto_VERSION VERSION_LESS "2.6.0")
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE config)
else()
  ament_auto_package(INSTALL_TO_SHARE config)
endif()

