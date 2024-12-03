cmake_minimum_required(VERSION 2.8.3)
project(path_searching)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
# set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# link_directories("/opt/ros/noetic/lib/x86_64-linux-gnu")
# link_directories(/opt/ros/noetic/lib/x86_64-linux-gnu)

# set(OMPL_INCLUDE_DIRS "/usr/local/include/ompl-1.6/")
# set(OMPL_LIBRARIES "/usr/local/lib/libompl.so")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(ompl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  visualization_msgs
  plan_env
  mm_config
  cv_bridge
  traj_utils
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES path_searching
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
    ${OMPL_INCLUDE_DIRS}
)

add_library(path_searching
    src/kino_astar.cpp
    src/sample_mani_RRT.cpp
    src/rrt.cpp
    )
target_link_libraries(path_searching
    ${catkin_LIBRARIES} 
    ${OMPL_LIBRARIES}
    )  
