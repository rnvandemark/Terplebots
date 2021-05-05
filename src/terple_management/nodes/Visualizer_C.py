#!/usr/bin/env python

import rospy
from terple_msgs.msg import BacktrackNode, MoveCommand, Path, AllPaths
import numpy as np
from cv2 import line, circle, VideoWriter, VideoWriter_fourcc
from math import sqrt, cos, sin, pi as PI


ROS_NODE_NAME = "centralized_visualizer"


def curve(xn, yn, theta_deg, ul, ur, dt=0.1, tf=1, img=None, scl=None):
    # img is an option arg. if present then func will also draw the curve on the img
    # Must also pass scl with image. represents how much bigger the output viz video is than the grid

    # reformat to be in gazebo grid system for calculations
    xn /= float(GRID_D)
    yn /= float(GRID_D)

    theta_rad = PI * theta_deg / 180.0  # convert deg to rad

    # draw 10 shorter lines to approximate the curve
    t = 0
    dist_traveled = 0
    while t < tf - dt / 2.0:
        t = t + dt  # step the time by increment dt
        xs = xn
        ys = yn

        # do the movement math
        dd = 0.5 * robot_r * (ul + ur) * dt
        dxn = dd * cos(theta_rad)
        dyn = dd * sin(theta_rad)
        xn += dxn
        yn += dyn
        theta_rad += ((robot_r / robot_L) * (ur - ul) * dt)
        dist_traveled += sqrt(dxn ** 2 + dyn ** 2)

        if img:
            img = line(img, (int(xs * scl * GRID_D), int(GRID_H * scl - ys * scl * GRID_D)),
                       (int(xn * scl * GRID_D), int(GRID_H * scl - yn * scl * GRID_D)),
                       (0, 0, 255))  # draw the small line, with scaled points

    # convert back to working dimensions
    theta_deg = 180 * theta_rad / PI
    theta_deg = theta_deg % 360
    xn *= GRID_D
    yn *= GRID_D
    dist_traveled *= GRID_D

    # return img only if one is given
    if img:
        return xn, yn, theta_deg, dist_traveled, img
    return xn, yn, theta_deg, dist_traveled, tf


def handle_path(msg):
    print "Path received. Starting render..."



def wait_for_paths():
    # Init ROS elements
    rospy.init_node(ROS_NODE_NAME)
    paths_sub = rospy.Subscriber(
        "/terple/all_paths",
        AllPaths,
        handle_path,
        queue_size=1
    )
    print "Ready, waiting for paths..."
    rospy.spin()


if __name__ == "__main__":
    # Init ROS elements and get parameters
    rospy.init_node(ROS_NODE_NAME)
    robot_r = rospy.get_param("/terple/robot_description/r")
    robot_L = rospy.get_param("/terple/robot_description/L")
    ROBOT_RADIUS = rospy.get_param("/terple/robot_description/ROBOT_RADIUS")
    CLEARANCE = rospy.get_param("/terple/space_description/CLEARANCE")
    BOARD_H = rospy.get_param("/terple/space_description/BOARD_H")
    BOARD_W = rospy.get_param("/terple/space_description/BOARD_W")
    BOARD_O = rospy.get_param("/terple/space_description/BOARD_O")
    GRID_D = rospy.get_param("/terple/space_description/GRID_D")
    WHEEL_SPEED_MAJOR = rospy.get_param("/terple/movement_description/WHEEL_SPEED_MAJOR")
    WHEEL_SPEED_MINOR = rospy.get_param("/terple/movement_description/WHEEL_SPEED_MINOR")
    SCALE = rospy.get_param("/terple/viz_description/SCALE")
    GRID_H = BOARD_H * GRID_D
    GRID_W = BOARD_W * GRID_D
    GRID_O = int(360 / BOARD_O)
    GRID_ROBOT_RADIUS = ROBOT_RADIUS * GRID_D
    GRID_CLEARANCE = CLEARANCE * GRID_D
    TOTAL_GRID_CLEARANCE = int(round(2 * GRID_ROBOT_RADIUS + GRID_CLEARANCE))  # point robot radius

    wait_for_paths()

"""    

    video_size = (GRID_W*scl, GRID_H*scl)
    # Build video writer to render the frames at 120 FPS
    video = cv2.VideoWriter(
       "visualization.mp4",
       cv2.VideoWriter_fourcc(*'mp4v'),
       30.0,
       video_size
    )

    # Build image to be white and draw the obstacles
    temp = np.uint8(setup_graph())
    temp *= 255
    img = np.empty((GRID_H, GRID_W, 3), dtype=np.uint8)
    img[:, :, 0] = temp
    img[:, :, 1] = temp
    img[:, :, 2] = temp
    img = cv2.resize(img, video_size, interpolation=cv2.INTER_NEAREST)

    # draw the explored nodes
    for i, p2d in enumerate(msg.explored):
        # draw each wheel speed action that comes off start node
        for action in p2d.moves_to_neighbors:
            ul = action.left_wheel_speed
            ur = action.right_wheel_speed
            img = draw_curve(img, p2d.position.x, p2d.position.y, p2d.position.theta, ul, ur, scl, (0, 0, 255))  # draws one move line

        # only write a fraction of the frames to keep video length down
        if i % 1 == 0:
            video.write(img)

    # draw the backtracking
    for i,curr in enumerate(msg.backtrack_path[:-1]):
        if curr.has_move_cmd:
            nxt = msg.backtrack_path[i+1]
            img = draw_curve(img, nxt.position.x, nxt.position.y, nxt.position.theta,
                             curr.move_cmd.left_wheel_speed, curr.move_cmd.right_wheel_speed,
                             scl, (0, 255, 0))  # draws one move line
            # only write a fraction of the frames to keep video length down
            if i % 1 == 0:
                video.write(img)

    video.release()
    print("Finished render.")
    
"""
