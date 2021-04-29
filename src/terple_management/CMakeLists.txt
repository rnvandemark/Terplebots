cmake_minimum_required(VERSION 3.0.2)
project(terple_management)

find_package(catkin REQUIRED COMPONENTS
  rospy
)

catkin_package(CATKIN_DEPENDS
  rospy
)
