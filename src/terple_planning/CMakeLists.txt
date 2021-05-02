cmake_minimum_required(VERSION 3.0.2)
project(terple_planning)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  gazebo_msgs
  terple_msgs
)

catkin_package(CATKIN_DEPENDS
)

catkin_install_python(PROGRAMS
  nodes/Planner_C.py
  nodes/Visualizer_C.py
  nodes/PlanRequester_C.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)