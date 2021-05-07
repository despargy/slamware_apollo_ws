cmake_minimum_required(VERSION 2.8.3)
project(apollo_lidar)


find_package(catkin REQUIRED COMPONENTS
  roscpp sensor_msgs message_filters
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES apollo_lidar
  CATKIN_DEPENDS roscpp sensor_msgs message_filters
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(apollo_lidar
  src/apollo_scan.cpp src/apollo_lidar_ros.cpp
)
add_dependencies(apollo_lidar ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(apollo_lidar
  ${catkin_LIBRARIES}
)
