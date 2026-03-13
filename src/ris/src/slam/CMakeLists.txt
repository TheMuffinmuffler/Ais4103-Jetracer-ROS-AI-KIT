add_executable(youssef_slam youssef_slam.cpp)
target_link_libraries(youssef_slam ${catkin_LIBRARIES})
add_dependencies(youssef_slam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(collision_guard collision_guard.cpp)
target_link_libraries(collision_guard ${catkin_LIBRARIES})
add_dependencies(collision_guard ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})