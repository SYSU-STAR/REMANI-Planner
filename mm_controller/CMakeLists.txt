cmake_minimum_required(VERSION 3.0.2)
project(mm_controller)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  control_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  trajectory_msgs
  quadrotor_msgs
  mm_utils
  traj_utils
)

catkin_package()

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} 
  # ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_library( controller_utils
  src/controller_utils.cpp
  src/mm_controller_fsm.cpp
)

target_link_libraries( controller_utils
  ${catkin_LIBRARIES}
)

add_executable( mm_controller_node
  src/mm_controller_node.cpp
)

target_link_libraries( mm_controller_node
  ${catkin_LIBRARIES}
  controller_utils
)

