cmake_minimum_required(VERSION 3.0.2)
project(move_control)

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

find_package (OpenCV REQUIRED)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_message_files(
	FILES
	velocity.msg
  my_control_frame.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	geometry_msgs
	actionlib_msgs
)

catkin_package(
  INCLUDE_DIRS include
  #LIBRARIES publisher
  CATKIN_DEPENDS roscpp std_msgs message_runtime geometry_msgs message_generation
  #DEPENDS system_lib
)

add_executable(move_control "")

target_sources(move_control 
PUBLIC
src/data_frame.cpp
src/Serial.cpp
src/move_control.cpp
)

add_dependencies(move_control ${${PROJECT_NAME}_EXPORTTED_TARGETS}
                                   ${catkin_EXPORTED_TARGETS}
                                   ${PROJECT_NAME}_generate_messages_cpp
                                   )
                      
target_link_libraries(move_control 
                      ${catkin_LIBRARIES} 
                      ${OpenCV_LIBRARIES} 
                      #${PROJECT_NAME}_generate_messages_cpp
                      )