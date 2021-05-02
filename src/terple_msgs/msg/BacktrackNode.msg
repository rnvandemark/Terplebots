# A position in the backtrack path
geometry_msgs/Pose2D position

# Whether or not the move command is populated
bool has_move_cmd

# The move command used to (ideally) get to this position
terple_msgs/MoveCommand move_cmd

# The time the node exists at
uint16 time