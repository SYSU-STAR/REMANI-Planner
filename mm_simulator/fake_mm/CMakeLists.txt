cmake_minimum_required(VERSION 3.0.2)
project(fake_mm)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
# ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  trajectory_msgs
  control_msgs
  sensor_msgs
  nav_msgs
  roslib
  roscpp
  rospy
  std_msgs
  tf
  mm_config
)

find_package(Eigen3 REQUIRED)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs tf
 DEPENDS mm_config
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable (fake_mm src/cmd_2_state.cpp )
target_link_libraries(fake_mm
  ${catkin_LIBRARIES})

add_executable (model_vis src/model_vis.cpp )
target_link_libraries(model_vis
  ${catkin_LIBRARIES})
