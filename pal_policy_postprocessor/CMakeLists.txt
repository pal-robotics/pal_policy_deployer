cmake_minimum_required(VERSION 3.0.2)
project(pal_policy_postprocessor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  message_filters
  visualization_msgs
  interactive_markers
)

catkin_package(
  CATKIN_DEPENDS
    roscpp std_msgs message_filters visualization_msgs interactive_markers
)


########################
include_directories(SYSTEM ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} /opt/onnxruntime/include)
include_directories(
  include
)
link_directories(/opt/onnxruntime/lib)
########################

add_executable(${PROJECT_NAME}_node src/postprocessor_kang_node.cpp)
target_link_libraries(${PROJECT_NAME}_node  
  ${catkin_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  onnxruntime
)

add_executable(${PROJECT_NAME}_balance_node src/postprocessor_kang_balance_node.cpp)
target_link_libraries(${PROJECT_NAME}_balance_node  
  ${catkin_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  onnxruntime
)

add_executable(${PROJECT_NAME}_loco_node src/postprocessor_kang_loco_node.cpp)
target_link_libraries(${PROJECT_NAME}_loco_node  
  ${catkin_LIBRARIES}
  ${Boost_FILESYSTEM_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  onnxruntime
)
##############


## Mark executables for installation
install(TARGETS  ${PROJECT_NAME}_balance_node ${PROJECT_NAME}_loco_node ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)