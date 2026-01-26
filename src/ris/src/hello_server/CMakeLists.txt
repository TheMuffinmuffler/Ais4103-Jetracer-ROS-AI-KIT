add_executable(hello_server
    main.cpp
)

add_dependencies(hello_server ${catkin_EXPORTED_TARGETS})

target_include_directories(hello_server PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(hello_server
    ${catkin_LIBRARIES}

    atomic
)

install(
    TARGETS hello_server
    EXPORT hello_server
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
