add_executable(slam_manager slam_manager.cpp)
target_link_libraries(slam_manager ${catkin_LIBRARIES})
add_dependencies(slam_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(slam_remote_saver slam_remote_saver.cpp)
target_link_libraries(slam_remote_saver ${catkin_LIBRARIES})
add_dependencies(slam_remote_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})