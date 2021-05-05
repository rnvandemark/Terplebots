cmake_minimum_required(VERSION 3.0.2)
project(terple_management)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  terple_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  rospy
  geometry_msgs
  terple_msgs
  std_msgs
)

catkin_install_python(PROGRAMS
  nodes/DecentralizedProgramOrchestrator.py
  nodes/DecentralizedSensingSimulator.py
  nodes/Manager_C.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
