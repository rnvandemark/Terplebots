# Whether or not a path was successfully planned
bool success

# The initial plan request, will be valid whether success is true or false
terple_msgs/PlanRequest request

# The order in which positions/nodes were explored, will be valid whether success
# is true or false, but will not reach the end if false
# nhr_msgs/NeighborsPose2D[] explored

# The path solution, from the final position to the initial position
# WARNING: the first node in this list (the last node in the list closest to the
# final position) might not be exactly equal to final_position, there's a finite
# discrepancy allowed. However, the last element in this list should be exactly
# equivalent to init_position. Will not be valid if success if false.
terple_msgs/BacktrackNode[] backtrack_path
