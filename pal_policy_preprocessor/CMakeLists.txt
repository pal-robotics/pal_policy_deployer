cmake_minimum_required(VERSION 3.8)
project(pal_policy_preprocessor)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node
  src/preprocessor.cpp
  src/preprocessor_tiago_reaching.cpp
)

ament_auto_package()

