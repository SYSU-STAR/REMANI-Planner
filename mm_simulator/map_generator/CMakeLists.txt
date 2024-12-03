cmake_minimum_required(VERSION 2.8.3)
project(map_generator)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  geometry_msgs
  pcl_conversions
)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

add_executable (map_generator src/map_generator.cpp )
target_link_libraries(map_generator
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES})
