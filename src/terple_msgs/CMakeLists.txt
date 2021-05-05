cmake_minimum_required(VERSION 3.0.2)
project(terple_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs)

add_message_files(FILES
  BacktrackNode.msg
  MoveCommand.msg
  NeighborsPose2D.msg
  Path.msg
  PlanRequest.msg
  AllPaths.msg
)

generate_messages(DEPENDENCIES geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)
