cmake_minimum_required(VERSION 2.8.3)
project(apollo_ros_sdk)

find_path(slamware_sdk_INCLUDE_DIR rpos/rpos.h ${PROJECT_SOURCE_DIR}/../slamware_sdk/include)
find_path(slamware_sdk_LIBRARY librpos_framework.a ${PROJECT_SOURCE_DIR}/../slamware_sdk/lib/linux-x86_64-gcc5.4)
if(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)
  set(SLTC_SDK_INC_DIR /home/despina/slamware_apollo_ws/src/slamware_sdk/include)
  set(SLTC_SDK_LIB_DIR "${slamware_sdk_LIBRARY}")
else(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)
  if(NOT DEFINED SLTC_SDK_ROOT OR SLTC_SDK_ROOT STREQUAL "")
    set(SLTC_SDK_ROOT "/home/slamware/mywd/slamtec/slamware_sdk_linux-x86_64-gcc5.4")
  endif()
  set(SLTC_SDK_DIR "${SLTC_SDK_ROOT}/linux-x86_64-release")
  set(SLTC_SDK_INC_DIR "${SLTC_SDK_DIR}/include")
  set(SLTC_SDK_LIB_DIR "${SLTC_SDK_DIR}/lib")
endif(slamware_sdk_INCLUDE_DIR AND slamware_sdk_LIBRARY)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -std=gnu++11")

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  message_generation
)

find_package(Threads)



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES slamware_ros_sdk
  CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs std_msgs tf message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
message("${INCLUDE_DIRECTORIES}")


add_executable(apollo_ros_sdk_server_node
  src/main.cpp src/ApolloRobot.cpp 
)
add_dependencies(apollo_ros_sdk_server_node ${PROJECT_NAME}_generate_messages_cpp)
target_include_directories(apollo_ros_sdk_server_node
  PRIVATE ${SLTC_SDK_INC_DIR}
)
target_compile_options(apollo_ros_sdk_server_node
  PRIVATE -Wno-deprecated-declarations
)
target_link_libraries(apollo_ros_sdk_server_node
  ${catkin_LIBRARIES}
  ${SLTC_SDK_LIB_DIR}/librpos_robotplatforms_rpslamware.a
  ${SLTC_SDK_LIB_DIR}/librpos_framework.a
  ${SLTC_SDK_LIB_DIR}/libbase64.a
  ${SLTC_SDK_LIB_DIR}/librlelib.a
  ${SLTC_SDK_LIB_DIR}/libjsoncpp.a
  ${SLTC_SDK_LIB_DIR}/libcurl.a
  ${SLTC_SDK_LIB_DIR}/libssl.a
  ${SLTC_SDK_LIB_DIR}/libcrypto.a
  ${SLTC_SDK_LIB_DIR}/libboost_atomic.a
  ${SLTC_SDK_LIB_DIR}/libboost_chrono.a
  ${SLTC_SDK_LIB_DIR}/libboost_date_time.a
  ${SLTC_SDK_LIB_DIR}/libboost_regex.a
  ${SLTC_SDK_LIB_DIR}/libboost_filesystem.a
  ${SLTC_SDK_LIB_DIR}/libboost_system.a
  ${SLTC_SDK_LIB_DIR}/libboost_thread.a
  ${SLTC_SDK_LIB_DIR}/libboost_random.a
  pthread
  dl
  rt
)
