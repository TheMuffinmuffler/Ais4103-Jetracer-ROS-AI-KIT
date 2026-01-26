add_executable(hello_world
    main.cpp
)

add_dependencies(hello_world ${catkin_EXPORTED_TARGETS})

target_include_directories(hello_world PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(hello_world
        ${catkin_LIBRARIES}

        atomic
)

install(
    TARGETS hello_world
    EXPORT hello_world
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
