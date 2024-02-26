cmake_minimum_required(VERSION 3.0.2)
project(messop_ugv)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  mess_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs mess_msgs nav_msgs rospy sensor_msgs std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/messop
  scripts/logger
  scripts/followline
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
