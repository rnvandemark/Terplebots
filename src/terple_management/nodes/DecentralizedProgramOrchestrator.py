#!/usr/bin/env python

import rospy
from terple_msgs.msg import DecentralizedProgram, DecentralizedRobotStatus, Vector2
from std_msgs.msg import Empty
from sys import argv as sargv

ROS_NODE_NAME = "terple_decentralized_program_orchestrator"

# A class to listen as the program progresses
class ProgramOrchestrator(object):

    # A dict to track the updates heard by the decentralized robots
    updates_by_id = None

    # Initialize with no entries
    def __init__(self, robot_ids):
        self.updates_by_id = {r:[False,False] for r in robot_ids}

    # Clear the update flags
    def clear_updates(self):
        self.updates_by_id = {r:[False,v[1]] for r,v in self.updates_by_id.items()}

    # Update the robot update and finished flags for the given ID
    def set_for_id(self, i, finished):
        self.updates_by_id[i] = [True, finished]

    # Check if all robots have been updated this cycle
    def all_ids_updated(self):
        return all(v[0] for v in self.updates_by_id.values())

    # Check if all robots have finished their motion
    def all_ids_finished(self):
        return all(v[1] for v in self.updates_by_id.values())

# Set the update flag in the given orchestrator
def update_callback(msg, orch, pub):
    orch.set_for_id(msg.robot_id, msg.finished)
    if orch.all_ids_finished():
        rospy.loginfo("All robots have finished.")
        rospy.signal_shutdown("Received path, ready for clean shutdown.")
    elif orch.all_ids_updated():
        orch.clear_updates()
        pub.publish(Empty())

def main():
    # Capture user input
    my_sargv = rospy.myargv(argv=sargv)

    robot_ids = []
    start_positions = []
    goal_positions = []
    for arg in my_sargv[1:]:
        robot_id, xi, yi, xf, yf = arg.split(",")
        robot_ids.append(int(robot_id))
        start_positions.append(Vector2(x=float(xi),y=float(yi)))
        goal_positions.append(Vector2(x=float(xf),y=float(yf)))

    # Create the orchestrator
    orch = ProgramOrchestrator(robot_ids)

    # Init ROS elements
    rospy.init_node(ROS_NODE_NAME)
    step_pub = rospy.Publisher(
        "/terple/decentralized/step",
        Empty,
        queue_size=1
    )
    characteristics_updates_sub = rospy.Subscriber(
        "/terple/decentralized/robot_status",
        DecentralizedRobotStatus,
        lambda m: update_callback(m, orch, step_pub),
        queue_size=100
    )
    programs_pub = rospy.Publisher(
        "/terple/decentralized/programs",
        DecentralizedProgram,
        queue_size=1
    )

    # Publish the program and wait for updates
    rospy.sleep(0.5)
    programs_pub.publish(DecentralizedProgram(
        robot_ids=robot_ids,
        start_positions=start_positions,
        goal_positions=goal_positions
    ))

    rospy.loginfo("Published program.")
    rospy.spin()

if __name__ == "__main__":
    main()
