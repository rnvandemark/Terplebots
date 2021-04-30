#!/usr/bin/env python

from __future__ import print_function
import rospy
from terple_msgs.msg import Path
import numpy as np
import cv2
from math import sqrt, cos, sin, pi as PI
import os

ROS_NODE_NAME = "centralized_visualizer"
GRID_H = 100
GRID_W = 100

def quad_check(x0, y0, quad):
    # extract coords out of quad list
    x1 = quad[0]
    y1 = quad[1]
    x2 = quad[2]
    y2 = quad[3]
    x3 = quad[4]
    y3 = quad[5]
    x4 = quad[6]
    y4 = quad[7]

    # check if the point is within the restricted half-plane side of each line
    chk1 = line_check(x0, y0, x1, y1, x2, y2, False, False)
    chk2 = line_check(x0, y0, x2, y2, x3, y3, False, True)
    chk3 = line_check(x0, y0, x3, y3, x4, y4, True, True)
    chk4 = line_check(x0, y0, x4, y4, x1, y1, True, False)

    # check if point is within restricted half place side of all lines --> in object
    return not (chk1 and chk2 and chk3 and chk4) # if True, point is not in obstacle space


def line_check(x0, y0, x1, y1, x2, y2, UD, LR):
    # UD = True  if object is bottom side of line
    # UD = False if object is top    side of line
    # LR = True  if object is left   side of line
    # LR = False if object is right  side of line
    if x2 != x1:  # not vertical line
        m = (y2 - y1) / float(x2 - x1)  # get the slope
        b = y1 - m * x1  # get the intercept
        # check if point is within the restricted half-plane
        return (y0 >= m * x0 + b and not UD) or (y0 <= m * x0 + b and UD) # if True, point is within the restricted half-plane
    else:  # x2 == x1 --> vertical line
        return (x0 >= x1 and not LR) or (x0 <= x1 and LR) # if True, point is within the restricted half-plane


def elip_check(x0, y0, elip):
    # extract dimensions out of elip list
    xc = elip[0]
    yc = elip[1]
    a2 = elip[2] ** 2  # horizontal dimension
    b2 = elip[3] ** 2  # vertical dimension
    return (x0 - xc) ** 2 / float(a2) + (y0 - yc) ** 2 / float(b2) > 1 # if True, point is in obstacle space


def setup_graph():
    quads = [[2.5, 57.5, 2.5, 42.5, 17.5, 42.5, 17.5, 57.5],
             [37.5, 57.5, 37.5, 42.5, 62.5, 42.5, 62.5, 57.5],
             [72.5, 40, 72.5, 20, 87.5, 20, 87.5, 40]]
    elips = [[20, 20, 10, 10],
             [20, 80, 10, 10]]

    obst = np.ones((GRID_H, GRID_W))
    for x in range(GRID_W):
        for y in range(GRID_H):
            for quad in quads:  # check quads
                if not quad_check(x, y, quad):  # see if point is near the quad
                    obst[GRID_H - y - 1, x] = 0
                    break

            for elip in elips:  # check elips
                if not elip_check(x, y, elip):  # see if point is near the elip
                    obst[GRID_H - y - 1, x] = 0
                    break

    return obst

def draw_curve(img, xn, yn, thetas, ul, ur, scl, color):
    r = 0.038  # wheel radius
    L = 0.354  # wheel base
    dt = 0.1   # changes number of lines

    # reformat to be in 10x10 grid system for calculations
    xn /= 10.0
    yn /= 10.0

    theta_rad = PI * thetas / 180.0  # convert deg to rad

    # draw 10 shorter lines to approximate the curve
    t = 0
    while t < 0.95:
        t = t + dt  # step the time by increment dt
        xs = xn  # start point x - set as previous goal
        ys = yn  # start point y - set as previous goal
        xn += 0.5 * r * (ul + ur) * cos(theta_rad) * dt  # goal point x
        yn += 0.5 * r * (ul + ur) * sin(theta_rad) * dt  # goal point y
        theta_rad += (r / float(L)) * (ur - ul) * dt  # goal theta
        img = cv2.line(img, (int(xs*scl*10), int(GRID_H*scl - ys*scl*10)), (int(xn*scl*10), int(GRID_H*scl - yn*scl*10)), color)  # draw the small line, with scaled points

    return img


def handle_path(msg):
    print("Path received. Starting render...")
    scl = 10  # output video scale factor
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


def main():
    # Init ROS elements
    rospy.init_node(ROS_NODE_NAME)
    path_sub = rospy.Subscriber(
        "/nhr/path",
        Path,
        handle_path,
        queue_size=1
    )
    print("Ready, waiting for paths...")
    rospy.spin()


if __name__ == "__main__":
    main()
