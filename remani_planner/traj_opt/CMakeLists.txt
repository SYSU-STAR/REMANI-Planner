cmake_minimum_required(VERSION 2.8.3)
project(traj_opt)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
#   cv_bridge
  path_searching
  mm_config
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_opt
 CATKIN_DEPENDS plan_env
 CATKIN_DEPENDS mm_config
 CATKIN_DEPENDS traj_utils
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
    # ${PROJECT_SOURCE_DIR}
)

add_library( traj_opt 
    src/poly_traj_optimizer.cpp
)

target_link_libraries( traj_opt
    ${catkin_LIBRARIES}
    ) 

