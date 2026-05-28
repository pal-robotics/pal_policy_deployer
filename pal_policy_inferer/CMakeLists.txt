cmake_minimum_required(VERSION 3.8)
project(pal_policy_inferer)

find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SwitchPolicy.action"
  DEPENDENCIES action_msgs
)

find_library(ONNXRUNTIME_LIB onnxruntime REQUIRED)
find_path(ONNXRUNTIME_INCLUDE_DIR onnxruntime_cxx_api.h
  PATH_SUFFIXES onnxruntime
  REQUIRED)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/inferer.cpp
  src/onnx.cpp
)
target_include_directories(${PROJECT_NAME}_node PRIVATE ${ONNXRUNTIME_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}_node ${ONNXRUNTIME_LIB})

ament_auto_add_executable(kang_mjlab_node src/kang_mjlab_node.cpp)
target_include_directories(kang_mjlab_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(kang_mjlab_node "${cpp_typesupport_target}")

ament_export_dependencies(rosidl_default_runtime)
ament_auto_package()
