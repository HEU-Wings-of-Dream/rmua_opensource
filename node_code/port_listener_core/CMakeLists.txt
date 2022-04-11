cmake_minimum_required(VERSION 3.0.2)
project(port_listener_core)

#catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib
	geometry_msgs
	actionlib_msgs
	message_runtime
	message_generation
	tf
)

add_message_files(
	FILES
	Posemsg.msg
  my_control_frame.msg
	velocity.msg
  race_state.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	actionlib_msgs
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES port_listener_core
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs message_generation
  #DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# catkin_install_python(PROGRAMS
# scripts/robottcp.py
# DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

add_executable(port_listener_core "")

target_sources(port_listener_core 
PUBLIC
src/source.cpp
src/src/serial.cc
src/src/impl/unix.cc
src/src/impl/list_ports/list_ports_linux.cc
)

add_executable(tcpcore "")

target_sources(tcpcore 
PUBLIC
src/tcpcore.cpp
)

add_dependencies(port_listener_core ${${PROJECT_NAME}_EXPORTTED_TARGETS}
                                   ${catkin_EXPORTED_TARGETS}
                                   ${PROJECT_NAME}_generate_messages_cpp
                                   #${PROJECT_NAME}_gencpp
                                   )

target_link_libraries(port_listener_core
						${catkin_LIBRARIES}
						#${PROJECT_NAME}_generate_messages_cpp

)


add_dependencies(tcpcore ${${PROJECT_NAME}_EXPORTTED_TARGETS}
                                   ${catkin_EXPORTED_TARGETS}
                                   ${PROJECT_NAME}_generate_messages_cpp
                                   #${PROJECT_NAME}_gencpp
                                   )

target_link_libraries(tcpcore
						${catkin_LIBRARIES}
						#${PROJECT_NAME}_generate_messages_cpp

)
