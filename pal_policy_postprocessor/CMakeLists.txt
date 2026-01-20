cmake_minimum_required(VERSION 3.8)
project(pal_policy_postprocessor)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(${PROJECT_NAME}_node src/postprocessor.cpp
src/postprocessor_tiago_reaching.cpp
)

ament_auto_package()