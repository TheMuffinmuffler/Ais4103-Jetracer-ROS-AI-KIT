add_subdirectory(hello_world)
add_subdirectory(map_reader)
add_subdirectory(graph_planner)
add_subdirectory(grid_planner)
add_subdirectory(slam)






add_library(ris_astar_global_planner
        navigation/astar_global_planner.cpp
)
target_link_libraries(ris_astar_global_planner
        ${catkin_LIBRARIES}
)

add_executable(simple_goal_sender
        navigation/simple_goal_sender.cpp
)
target_link_libraries(simple_goal_sender
        ${catkin_LIBRARIES}
)

add_executable(initial_pose_sender
        navigation/initial_pose_sender.cpp
)
target_link_libraries(initial_pose_sender
        ${catkin_LIBRARIES}
)

install(TARGETS
        ris_astar_global_planner
        simple_goal_sender
        initial_pose_sender
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)