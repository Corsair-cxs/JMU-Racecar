cmake_minimum_required(VERSION 2.8.3)
project(jmu_racecar_joystick)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  robotnik_msgs
  roscpp
  sensor_msgs
  diagnostic_msgs
  diagnostic_updater
  std_srvs
  std_msgs
  ackermann_msgs
)


catkin_package(
	CATKIN_DEPENDS robotnik_msgs ackermann_msgs
 )


include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(racecar_joystick_node src/racecar_joystick.cpp)


# add_dependencies(racecar_joystick_node rbcar_joystick_generate_messages_cpp)
# add_dependencies(racecar_joystick_node robotnik_msgs_generate_messages_cpp)


target_link_libraries(racecar_joystick_node
   ${catkin_LIBRARIES}
)
