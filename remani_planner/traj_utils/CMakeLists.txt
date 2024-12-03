cmake_minimum_required(VERSION 2.8.3)
project(traj_utils)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
# set(ENABLE_PRECOMPILED_HEADERS "OFF")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  message_generation
  mm_config
)

# find_package(Eigen3 REQUIRED)
# find_package(PCL 1.7 REQUIRED) #to help find Eigen

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  DataDisp.msg
  PolyTraj.msg
  Assignment.msg
  )
  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES traj_utils 
 CATKIN_DEPENDS message_runtime
 CATKIN_DEPENDS mm_config
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    # ${EIGEN3_INCLUDE_DIR}
)

# link_directories(${PCL_LIBRARY_DIRS})

add_library( traj_utils
    src/main.cpp 
    )
target_link_libraries( traj_utils
    ${catkin_LIBRARIES} 
    )  
