cmake_minimum_required(VERSION 3.10)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        nav_core
        costmap_2d
        pluginlib
        geometry_msgs
        nav_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        std_msgs
)

catkin_package(
        INCLUDE_DIRS .
        LIBRARIES astar_global_planner
        CATKIN_DEPENDS
        roscpp
        nav_core
        costmap_2d
        pluginlib
        geometry_msgs
        nav_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        std_msgs
)

include_directories(
        .
        ${catkin_INCLUDE_DIRS}
)

add_library(astar_global_planner
        astar_global_planner.cpp
)

add_dependencies(astar_global_planner ${catkin_EXPORTED_TARGETS})

target_link_libraries(astar_global_planner
        ${catkin_LIBRARIES}
)

add_executable(initial_pose_sender initial_pose_sender.cpp)
add_dependencies(initial_pose_sender ${catkin_EXPORTED_TARGETS})
target_link_libraries(initial_pose_sender ${catkin_LIBRARIES})

add_executable(simple_goal_sender simple_goal_sender.cpp)
add_dependencies(simple_goal_sender ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_goal_sender ${catkin_LIBRARIES})

install(TARGETS
        astar_global_planner
        initial_pose_sender
        simple_goal_sender
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
        astar_global_planner_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)