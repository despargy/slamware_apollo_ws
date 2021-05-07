cmake_minimum_required(VERSION 3.0.2)
project(ap0_com)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  message_runtime
  move_base_msgs
  roscpp
  std_msgs
  tf2_ros
)

find_package(Boost REQUIRED COMPONENTS system)

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   apMove.action
# )

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

#add_message_files(FILES apollo.msg)
#add_service_files(FILES GetPermission.srv)



catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ap0_com
   CATKIN_DEPENDS actionlib actionlib_msgs tf2_ros  message_runtime move_base_msgs roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_executable(apollo_master_interrupt
  src/apollo_master_interrupt.cpp src/ActionsNSockets.cpp src/SocketClient.cpp
)

#add_executable(srv_interrupt
#  src/srv_interrupt.cpp src/SocketServer.cpp
#)


add_dependencies(apollo_master_interrupt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(apollo_master_interrupt
  ${catkin_LIBRARIES}
)
#add_dependencies(srv_interrupt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(srv_interrupt
#  ${catkin_LIBRARIES}
#)
