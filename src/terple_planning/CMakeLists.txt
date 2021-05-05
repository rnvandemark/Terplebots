cmake_minimum_required(VERSION 3.0.2)
project(terple_planning)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  terple_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  rospy
  terple_msgs
  std_msgs
)

catkin_install_python(PROGRAMS
  nodes/DecentralizedRobotPlanner.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)