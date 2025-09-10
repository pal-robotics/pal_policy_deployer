cmake_minimum_required(VERSION 3.0.2)
project(pal_policy_deployer)

find_package(catkin REQUIRED COMPONENTS
  pal_policy_inferer
  pal_policy_postprocessor
  pal_policy_preprocessor
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

