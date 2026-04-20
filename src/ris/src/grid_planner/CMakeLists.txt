
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  geometry_msgs
)

add_executable(grid_planner plan_testRviz2.cpp)
add_dependencies(grid_planner ${catkin_EXPORTED_TARGETS})
target_include_directories(grid_planner PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(grid_planner ${catkin_LIBRARIES})
