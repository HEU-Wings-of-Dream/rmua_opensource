cmake_minimum_required(VERSION 3.0.2)
project(mdvs_autoaim)

add_compile_options(-std=c++11)

SET(MDVS_SO /lib/libMVSDK.so)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

find_package (OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mdvs_autoaim
  CATKIN_DEPENDS actionlib geometry_msgs roscpp rospy std_msgs
  #DEPENDS system_lib
)

include_directories("${PROJECT_SOURCE_DIR}/include")

include_directories (
	include
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

add_executable(autoaim "")

target_sources(autoaim
PUBLIC
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

target_link_libraries(autoaim ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${MDVS_SO}
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -msse3 -std=c++11")