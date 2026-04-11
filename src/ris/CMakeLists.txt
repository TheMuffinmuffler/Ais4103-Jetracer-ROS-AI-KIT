cmake_minimum_required(VERSION 3.10)
project(ris)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

add_compile_options(-fPIC -Wall -Wextra -pedantic)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        roslib
        ris_msgs
        std_msgs
        std_srvs
        message_runtime
        nav_msgs
        geometry_msgs
        sensor_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        nav_core
        costmap_2d
        pluginlib
)

catkin_package(
        CATKIN_DEPENDS
        roscpp
        roslib
        ris_msgs
        std_msgs
        std_srvs
        message_runtime
        nav_msgs
        geometry_msgs
        sensor_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        nav_core
        costmap_2d
        pluginlib
)

include_directories(
        ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

install(FILES
        plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)