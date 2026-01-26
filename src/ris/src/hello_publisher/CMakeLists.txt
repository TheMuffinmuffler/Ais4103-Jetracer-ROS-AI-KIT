add_executable(hello_publisher
    main.cpp
)

add_dependencies(hello_publisher ${catkin_EXPORTED_TARGETS})

target_include_directories(hello_publisher PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(hello_publisher
    ${catkin_LIBRARIES}

    atomic
)

install(
    TARGETS hello_publisher
    EXPORT hello_publisher
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
