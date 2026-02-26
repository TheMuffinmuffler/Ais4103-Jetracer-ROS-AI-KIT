
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
)


add_executable(map_reader
    main.cpp
)

add_dependencies(map_reader ${catkin_EXPORTED_TARGETS})

target_include_directories(map_reader PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(map_reader
    ${catkin_LIBRARIES}

    atomic
)

install(
    TARGETS map_reader
    EXPORT map_reader
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
