cmake_minimum_required(VERSION 3.0.2)
project(slam_pkg)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(smart_slam_node src/smart_slam.cpp)
target_link_libraries(smart_slam_node
  ${catkin_LIBRARIES}
)
