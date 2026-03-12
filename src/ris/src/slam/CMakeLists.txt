add_executable(youssef_slam youssef_slam.cpp)
target_link_libraries(youssef_slam ${catkin_LIBRARIES})
add_dependencies(youssef_slam ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})