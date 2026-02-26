# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ris_msgs: 1 messages, 1 services")

set(MSG_I_FLAGS "-Iris_msgs:/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/msg/;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ris_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/msg//Hello.msg" NAME_WE)
add_custom_target(_ris_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ris_msgs" "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/msg//Hello.msg" ""
)

get_filename_component(_filename "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/srv//PingPong.srv" NAME_WE)
add_custom_target(_ris_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ris_msgs" "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/srv//PingPong.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ris_msgs
  "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/msg//Hello.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ris_msgs
)

### Generating Services
_generate_srv_cpp(ris_msgs
  "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/srv//PingPong.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ris_msgs
)

### Generating Module File
_generate_module_cpp(ris_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ris_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ris_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ris_msgs_generate_messages ris_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/msg//Hello.msg" NAME_WE)
add_dependencies(ris_msgs_generate_messages_cpp _ris_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/srv//PingPong.srv" NAME_WE)
add_dependencies(ris_msgs_generate_messages_cpp _ris_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ris_msgs_gencpp)
add_dependencies(ris_msgs_gencpp ris_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ris_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ris_msgs
  "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/msg//Hello.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ris_msgs
)

### Generating Services
_generate_srv_eus(ris_msgs
  "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/srv//PingPong.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ris_msgs
)

### Generating Module File
_generate_module_eus(ris_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ris_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ris_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ris_msgs_generate_messages ris_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/msg//Hello.msg" NAME_WE)
add_dependencies(ris_msgs_generate_messages_eus _ris_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/srv//PingPong.srv" NAME_WE)
add_dependencies(ris_msgs_generate_messages_eus _ris_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ris_msgs_geneus)
add_dependencies(ris_msgs_geneus ris_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ris_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ris_msgs
  "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/msg//Hello.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ris_msgs
)

### Generating Services
_generate_srv_lisp(ris_msgs
  "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/srv//PingPong.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ris_msgs
)

### Generating Module File
_generate_module_lisp(ris_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ris_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ris_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ris_msgs_generate_messages ris_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/msg//Hello.msg" NAME_WE)
add_dependencies(ris_msgs_generate_messages_lisp _ris_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/srv//PingPong.srv" NAME_WE)
add_dependencies(ris_msgs_generate_messages_lisp _ris_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ris_msgs_genlisp)
add_dependencies(ris_msgs_genlisp ris_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ris_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ris_msgs
  "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/msg//Hello.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ris_msgs
)

### Generating Services
_generate_srv_nodejs(ris_msgs
  "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/srv//PingPong.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ris_msgs
)

### Generating Module File
_generate_module_nodejs(ris_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ris_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ris_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ris_msgs_generate_messages ris_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/msg//Hello.msg" NAME_WE)
add_dependencies(ris_msgs_generate_messages_nodejs _ris_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/srv//PingPong.srv" NAME_WE)
add_dependencies(ris_msgs_generate_messages_nodejs _ris_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ris_msgs_gennodejs)
add_dependencies(ris_msgs_gennodejs ris_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ris_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ris_msgs
  "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/msg//Hello.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ris_msgs
)

### Generating Services
_generate_srv_py(ris_msgs
  "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/srv//PingPong.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ris_msgs
)

### Generating Module File
_generate_module_py(ris_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ris_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ris_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ris_msgs_generate_messages ris_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/msg//Hello.msg" NAME_WE)
add_dependencies(ris_msgs_generate_messages_py _ris_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jetson/Desktop/Ais4103-Jetracer-ROS-AI-KIT/src/ris_msgs/srv//PingPong.srv" NAME_WE)
add_dependencies(ris_msgs_generate_messages_py _ris_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ris_msgs_genpy)
add_dependencies(ris_msgs_genpy ris_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ris_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ris_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ris_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ris_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ris_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ris_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ris_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ris_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ris_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ris_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ris_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ris_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ris_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ris_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ris_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ris_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ris_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
