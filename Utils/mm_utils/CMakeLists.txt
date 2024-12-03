cmake_minimum_required(VERSION 2.8.3)
project(mm_utils)

## Turn on verbose output
set(CMAKE_VERBOSE_MAKEFILE "false")

## Enable C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

## Add some compile flags
set(ADDITIONAL_CXX_FLAG "-Wall -O3 -g")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${ADDITIONAL_CXX_FLAG}")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  # cmake_utils
)


find_package(Eigen3 REQUIRED) # try to find manually installed eigen (Usually in /usr/local with provided FindEigen3.cmake)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  # LIBRARIES mm_utils
  CATKIN_DEPENDS roscpp rospy
  # DEPENDS system_lib
)

###########
## Build ##
###########


include_directories(
  include
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test src/${PROJECT_NAME}_test.cpp)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()
