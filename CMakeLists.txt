cmake_minimum_required(VERSION 3.0.2)
project(messop_ugv)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  std_msgs
  message_generation
)

add_message_files(
  FILES
  MessToUGV.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/messop
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

