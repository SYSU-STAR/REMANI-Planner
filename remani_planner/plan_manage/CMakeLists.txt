cmake_minimum_required(VERSION 2.8.3)
project(remani_planner)

set(CMAKE_BUILD_TYPE "Release")

ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  quadrotor_msgs
  plan_env
  path_searching
  traj_opt
  traj_utils
  message_generation
  cv_bridge
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES remani_planner
 CATKIN_DEPENDS plan_env path_searching traj_opt traj_utils 
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

add_library( vis_utils
    src/planning_visualization.cpp 
    )
target_link_libraries( vis_utils
    ${catkin_LIBRARIES} 
    )  

add_executable(remani_planner_node
  src/remani_planner_node.cpp 
  src/remani_replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(remani_planner_node 
  ${catkin_LIBRARIES}
  vis_utils
  )


