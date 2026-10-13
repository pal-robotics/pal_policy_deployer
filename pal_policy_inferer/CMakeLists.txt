cmake_minimum_required(VERSION 3.8)
project(pal_policy_inferer)

find_package(ament_cmake_auto REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(onnxruntime_vendor REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SwitchPolicy.action"
  DEPENDENCIES action_msgs
)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/inferer.cpp
  src/onnx.cpp
)
target_link_libraries(${PROJECT_NAME}_node onnxruntime::onnxruntime)
# Ensure onnxruntime library can be found at runtime regardless of how it was installed above.
# onnxruntime_vendor installs libonnxruntime.so under opt/onnxruntime_vendor/lib/ relative to the
# install prefix, so that directory must be in the RPATH alongside the standard lib/.
set_target_properties(${PROJECT_NAME}_node PROPERTIES
  INSTALL_RPATH "\$ORIGIN/../lib;\$ORIGIN/../opt/onnxruntime_vendor/lib;${CMAKE_INSTALL_PREFIX}/lib"
)

# ROS/ONNX-free policy types and helpers, split out so they can be unit tested.
ament_auto_add_library(kang_mjlab_policy SHARED src/kang_mjlab_policy.cpp)

ament_auto_add_executable(kang_mjlab_node src/kang_mjlab_node.cpp)
target_link_libraries(kang_mjlab_node kang_mjlab_policy)
target_include_directories(kang_mjlab_node PRIVATE ${EIGEN3_INCLUDE_DIRS})
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(kang_mjlab_node "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_kang_mjlab_policy test/test_kang_mjlab_policy.cpp)
  target_link_libraries(test_kang_mjlab_policy kang_mjlab_policy)
endif()

ament_export_dependencies(rosidl_default_runtime)
if(ament_cmake_auto_VERSION VERSION_LESS "2.6.0")
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
else()
  ament_auto_package()
endif()