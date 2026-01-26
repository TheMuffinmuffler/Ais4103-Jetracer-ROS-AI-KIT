cmake_minimum_required(VERSION 3.10)
project(ris)

## Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_VERBOSE_MAKEFILE ON)

## Add compile options
add_compile_options(-fPIC -Wall -pedantic)

## 1. Find ROS Packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  ris_msgs
  std_msgs
  message_runtime
)

## 2. Declare Include Directories (CRITICAL)
## This tells the compiler where to find ros/ros.h
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## 3. Catkin Package Configuration
catkin_package(
  CATKIN_DEPENDS roscpp ris_msgs std_msgs
)

## 4. Traverse into the src folder
add_subdirectory(src)