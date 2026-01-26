add_executable(hello_client
    main.cpp
)

add_dependencies(hello_client ${catkin_EXPORTED_TARGETS})

target_include_directories(hello_client PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(hello_client
    ${catkin_LIBRARIES}

    atomic
)

install(
    TARGETS hello_client
    EXPORT hello_client
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
