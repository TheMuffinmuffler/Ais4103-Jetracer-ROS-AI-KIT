add_executable(hello_subscriber
    main.cpp
)

add_dependencies(hello_subscriber ${catkin_EXPORTED_TARGETS})

target_include_directories(hello_subscriber PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} SYSTEM ${catkin_INCLUDE_DIRS})

target_link_libraries(hello_subscriber
    ${catkin_LIBRARIES}

    atomic
)

install(
    TARGETS hello_subscriber
    EXPORT hello_subscriber
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
