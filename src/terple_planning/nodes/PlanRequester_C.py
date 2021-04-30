#!/usr/bin/env python

import rospy
from terple_msgs.msg import PlanRequest, Path
from geometry_msgs.msg import Pose2D, Twist, Pose, Point, Quaternion
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from sys import argv as sargv
from math import pi as PI

ROS_NODE_NAME = "centralized_plan_requester"


# Get a ROS parameter from the server if it exists
def get_rosparam(name):
    value = None
    if rospy.has_param(name):
        value = rospy.get_param(name)
    return value


# Populate a twist message given motion of the wheels
def get_twist_from(Ul, Ur, r, L):
    twist = Twist()
    twist.linear.x = (r / 2) * (Ul + Ur)
    twist.angular.z = (r / L) * (Ur - Ul)
    return twist


# Publish a series of twist messages given a path to follow
def handle_path(msg, wheel_radius, lateral_separation, gazebo_model_cli, twist_pub):
    if not msg.success:
        print "Plan failed!"

    # Issue service request to manually set the position/orientation of the turtlebot with no twist
    srv_response = gazebo_model_cli(model_state=ModelState(
        model_name="turtlebot3_burger",
        pose=Pose(
            position=Point(
                x=(msg.request.init_position.x) - 5,  # Recenter, as the origin is at (5,5)
                y=(msg.request.init_position.y) - 5,  # Recenter, as the origin is at (5,5)
                z=0
            ),
            orientation=Quaternion(x=0, y=0, z=0, w=1)  # Identity quaternion
        ),
        twist=Twist(),
        reference_frame="world"  # "world" or "map" is recognized as the 'world' frame
    ))
    assert (srv_response.success)
    rospy.sleep(1)

    # Iterate through the generated move commands (reverse the backtrack path to move forward)
    move_commands = (p.move_cmd for p in reversed(msg.backtrack_path) if p.has_move_cmd)
    for cmd in move_commands:
        twist_pub.publish(get_twist_from(
            cmd.left_wheel_speed,
            cmd.right_wheel_speed,
            wheel_radius,
            lateral_separation
        ))
        rospy.sleep(cmd.time_elapsed)  # Wait for the amount of time that the bot should move
    twist_pub.publish(Twist())  # Publish 0 twist to stop t=he bot
    rospy.signal_shutdown("Received path, ready for clean shutdown.")


def main():
    # Capture required user input
    my_sargv = rospy.myargv(argv=sargv)
    assert (len(my_sargv) == 7)

    ii = None; ij = None; fi = None; fj = None; w1 = None; w2 = None
    try:
        ii = float(my_sargv[1])
        ij = float(my_sargv[2])
        fi = float(my_sargv[3])
        fj = float(my_sargv[4])
    except:
        print "Input coordinates must be numbers."
        return
    try:
        w1 = int(my_sargv[5])
        w2 = int(my_sargv[6])
    except:
        print "Input wheel speeds must be positive integers."
        return
    if (w1 <= 0) or (w2 <= 0):
        print "Input wheel speeds must be positive integers."
        return

    # Init ROS elements
    rospy.init_node(ROS_NODE_NAME)
    robot_r = get_rosparam("/nhr/robot_description/r")
    robot_L = get_rosparam("/nhr/robot_description/L")
    assert ((robot_r != None) and (robot_L != None))

    rospy.wait_for_service("/gazebo/set_model_state")
    model_state_cli = rospy.ServiceProxy(
        "/gazebo/set_model_state",
        SetModelState
    )
    path_pub = rospy.Publisher(
        "/nhr/plan_request",
        PlanRequest,
        queue_size=1
    )
    cmd_vel_pub = rospy.Publisher(
        "/cmd_vel",
        Twist,
        queue_size=1
    )
    path_sub = rospy.Subscriber(
        "/nhr/path",
        Path,
        lambda m: handle_path(m, robot_r, robot_L, model_state_cli, cmd_vel_pub),
        queue_size=1
    )

    sleep_time_s = 0.5
    print "First sleeping for {0} second(s)...".format(sleep_time_s)
    rospy.sleep(sleep_time_s)
    path_pub.publish(PlanRequest(
        init_position=Pose2D(x=ii, y=ij, theta=0),
        final_position=Pose2D(x=fi, y=fj, theta=0),
        wheel_speed_minor=w1,
        wheel_speed_major=w2
    ))
    print "Plan request published."
    rospy.spin()


if __name__ == "__main__":
    main()
