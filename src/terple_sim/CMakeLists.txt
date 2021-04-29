cmake_minimum_required(VERSION 3.0.2)
project(terple_sim)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  turtlebot3_description
)

catkin_package(CATKIN_DEPENDS
  gazebo_ros
  turtlebot3_description
)

install(
  DIRECTORY launch world
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
