cmake_minimum_required(VERSION 3.0.2)
project(terple_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs std_msgs)

add_message_files(FILES
  AllPaths.msg
  BacktrackNode.msg
  DecentralizedCombinedPaths.msg
  DecentralizedPath.msg
  DecentralizedProgram.msg
  DecentralizedRobotReadings.msg
  DecentralizedRobotStatus.msg
  MoveCommand.msg
  NeighborsPose2D.msg
  Path.msg
  PlanRequest.msg
  RobotExternalCharacteristics.msg
  Vector2.msg
)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)
