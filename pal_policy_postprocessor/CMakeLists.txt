cmake_minimum_required(VERSION 3.8)
project(pal_policy_postprocessor)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node src/postprocessor.cpp
src/postprocessor_tiago_reaching.cpp
)

if(ament_cmake_auto_VERSION VERSION_LESS "2.6.0")
  ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR INSTALL_TO_SHARE config)
else()
  ament_auto_package(INSTALL_TO_SHARE config)
endif()