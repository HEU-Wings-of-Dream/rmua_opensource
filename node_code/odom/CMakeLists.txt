project(odom)

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

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)

add_message_files(
	FILES
	Posemsg.msg
    my_control_frame.msg
	velocity.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES publisher
  CATKIN_DEPENDS roscpp std_msgs message_runtime geometry_msgs message_generation tf
  #DEPENDS system_lib
)


                                   
add_executable(odom src/odometry.cpp)

add_executable(odom_tf_publisher src/tf_publisher.cpp)

add_executable(amcl_listener src/amcl_reader.cpp)

add_executable(portlistener "")
target_sources(portlistener
PUBLIC
#src/data_frame.cpp
src/Serial.cpp
src/portlistener.cpp
)

add_dependencies(odom_tf_publisher ${${PROJECT_NAME}_EXPORTTED_TARGETS}
                                   ${catkin_EXPORTED_TARGETS}
                                   ${PROJECT_NAME}_generate_messages_cpp
                                   #${PROJECT_NAME}_gencpp
                                   )

add_dependencies(odom ${${PROJECT_NAME}_EXPORTTED_TARGETS}
                                   ${catkin_EXPORTED_TARGETS}
                                   ${PROJECT_NAME}_generate_messages_cpp
                                   #${PROJECT_NAME}_gencpp
                                   )
add_dependencies(portlistener ${${PROJECT_NAME}_EXPORTTED_TARGETS}
                                   ${catkin_EXPORTED_TARGETS}
                                   ${PROJECT_NAME}_generate_messages_cpp
                                   #${PROJECT_NAME}_gencpp
                                   )

add_dependencies(amcl_listener ${${PROJECT_NAME}_EXPORTTED_TARGETS}
                                   ${catkin_EXPORTED_TARGETS}
                                   ${PROJECT_NAME}_generate_messages_cpp
                                   #${PROJECT_NAME}_gencpp
                                   )

target_link_libraries(odom 
						${catkin_LIBRARIES} 
						#${PROJECT_NAME}_generate_messages_cpp
)
target_link_libraries(portlistener
						${catkin_LIBRARIES}
						#${PROJECT_NAME}_generate_messages_cpp

)
target_link_libraries(odom_tf_publisher
						${catkin_LIBRARIES}
						#${PROJECT_NAME}_generate_messages_cpp

)
target_link_libraries(amcl_listener
						${catkin_LIBRARIES}
						#${PROJECT_NAME}_generate_messages_cpp

)
