
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

add_executable(graph_planner main.cpp)
add_dependencies(graph_planner ${catkin_EXPORTED_TARGETS})
target_include_directories(graph_planner PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(graph_planner ${catkin_LIBRARIES})
