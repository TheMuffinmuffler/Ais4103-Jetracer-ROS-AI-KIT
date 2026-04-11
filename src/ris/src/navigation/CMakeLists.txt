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

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_library(astar_global_planner
        src/astar_global_planner.cpp
        src/grid_utils.cpp
        src/planner_utils.cpp
        src/pose_utils.cpp
)

add_dependencies(astar_global_planner ${catkin_EXPORTED_TARGETS})

target_link_libraries(astar_global_planner
        ${catkin_LIBRARIES}
)

add_executable(initial_pose_sender
        ../tools/initial_pose_sender.cpp
)
add_dependencies(initial_pose_sender ${catkin_EXPORTED_TARGETS})
target_link_libraries(initial_pose_sender ${catkin_LIBRARIES})

add_executable(simple_goal_sender
        ../tools/simple_goal_sender.cpp
)
add_dependencies(simple_goal_sender ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_goal_sender ${catkin_LIBRARIES})

add_executable(nav_status_monitor
        ../tools/nav_status_monitor.cpp
)
add_dependencies(nav_status_monitor ${catkin_EXPORTED_TARGETS})
target_link_libraries(nav_status_monitor ${catkin_LIBRARIES})

add_executable(pose_recorder
        ../tools/pose_recorder.cpp
)
add_dependencies(pose_recorder ${catkin_EXPORTED_TARGETS})
target_link_libraries(pose_recorder ${catkin_LIBRARIES})

install(TARGETS
        astar_global_planner
        initial_pose_sender
        simple_goal_sender
        nav_status_monitor
        pose_recorder
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)