# The path solution, from the final position to the initial position
# WARNING: the first node in this list (the last node in the list closest to the
# final position) might not be exactly equal to final_position, there's a finite
# discrepancy allowed. However, the last element in this list should be exactly
# equivalent to init_position. Will not be valid if success if false.
# NOTE: When used as part of AllPaths.msg, the path solution is reversed to be from initial to final
terple_msgs/BacktrackNode[] backtrack_path
