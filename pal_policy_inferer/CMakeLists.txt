cmake_minimum_required(VERSION 3.8)
project(pal_policy_inferer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(SYSTEM include 
  /opt/onnxruntime/include
)
link_directories(/opt/onnxruntime/lib)

ament_auto_add_executable(${PROJECT_NAME}_node src/onnx.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  onnxruntime
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
