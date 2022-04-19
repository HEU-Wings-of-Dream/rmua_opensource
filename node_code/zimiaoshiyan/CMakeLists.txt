cmake_minimum_required(VERSION 3.0.2)
project(zimiaoshiyan)

add_compile_options(-std=c++11)

set(V4L2LIB /so)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  roscpp
  rospy
  std_msgs
  geometry_msgs
)

find_package (OpenCV REQUIRED)

include_directories (
	include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	/usr/lib/x86_64-linux-gnu
)

catkin_package(
)

add_executable(zimiao "")
target_sources(zimiao
PUBLIC
src/Others/Camera/RMVideoCapture.cpp
src/Others/DataFrame/data_frame.cpp
src/Others/SolvePosition/solve_position.cpp
src/Others/Trajectory/trajectory.cpp
src/Serials/Serial.cpp

src/Armor/armor.cpp
#src/Armor/ArmorNum/armor_num.cpp
src/Armor/ArmorPredict/armor_predict.cpp
src/Armor/ArmorTrack/armor_track.cpp
src/Armor/FindLightBar/find_light_bar.cpp
src/Armor/RectangleMacth/rectangle_match.cpp

src/Rune/WindPredict/wind_predict.cpp
src/Rune/Wind.cpp

src/Run/run.cpp
src/main.cpp
)
link_directories(
	${V4L2LIB}
)
target_link_libraries(zimiao ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} 
libv4l2.so
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -msse3 -std=c++11")