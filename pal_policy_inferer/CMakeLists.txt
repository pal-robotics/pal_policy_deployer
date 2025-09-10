cmake_minimum_required(VERSION 3.0.2)
project(pal_policy_inferer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_generation
  message_filters
  visualization_msgs
  interactive_markers
)
find_package(Boost REQUIRED COMPONENTS filesystem system)

add_message_files(
  FILES
  StampedInt8Array.msg
  StampedFloat64Array.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime message_filters
)

########################
include_directories(SYSTEM ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} /opt/onnxruntime/include)
include_directories(
  include
)
link_directories(/opt/onnxruntime/lib)
########################



add_executable(${PROJECT_NAME}_reach_node src/inferer_kang_reach_node.cpp)
target_link_libraries(${PROJECT_NAME}_reach_node
  ${catkin_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  onnxruntime
)

add_executable(${PROJECT_NAME}_balance_node src/inferer_kang_balance_node.cpp)
target_link_libraries(${PROJECT_NAME}_balance_node
  ${catkin_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  onnxruntime
)

add_executable(${PROJECT_NAME}_loco_node src/inferer_kang_loco_node.cpp)
target_link_libraries(${PROJECT_NAME}_loco_node
  ${catkin_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  onnxruntime
)



install(TARGETS ${PROJECT_NAME}_reach_node ${PROJECT_NAME}_balance_node ${PROJECT_NAME}_loco_node  
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)