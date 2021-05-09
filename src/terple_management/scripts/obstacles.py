#!/usr/bin/env python

import numpy as np
from math import sqrt
from cv2 import circle


# ROBOT_RADIUS = rospy.get_param("/terple/robot_description/ROBOT_RADIUS")
# CLEARANCE = rospy.get_param("/terple/space_description/CLEARANCE")
# BOARD_H = rospy.get_param("/terple/space_description/BOARD_H")
# BOARD_W = rospy.get_param("/terple/space_description/BOARD_W")
# GRID_D = rospy.get_param("/terple/space_description/GRID_D")

# Board Obstacles
quads = [[]]
elips = [[]]


def quad_check(x0, y0, quad):
    # extract cords out of quad list
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
    return not (chk1 and chk2 and chk3 and chk4)  # if True, point is not in obstacle space


def line_check(x0, y0, x1, y1, x2, y2, UD, LR):
    # UD = True  if object is bottom side of line
    # UD = False if object is top    side of line
    # LR = True  if object is left   side of line
    # LR = False if object is right  side of line
    if x2 != x1:  # not vertical line
        m = (y2 - y1) / float(x2 - x1)  # get the slope
        b = y1 - m * x1  # get the intercept
        # check if point is within the restricted half-plane
        return (y0 >= m * x0 + b and not UD) or (
                    y0 <= m * x0 + b and UD)  # if True, point is within the restricted half-plane
    else:  # x2 == x1 --> vertical line
        return (x0 >= x1 and not LR) or (x0 <= x1 and LR)  # if True, point is within the restricted half-plane


def elip_check(x0, y0, elip):
    # extract dimensions out of elip list
    xc = elip[0]
    yc = elip[1]
    a2 = elip[2] ** 2  # horizontal dimension
    b2 = elip[3] ** 2  # vertical dimension
    return (x0 - xc) ** 2 / float(a2) + (y0 - yc) ** 2 / float(b2) > 1  # if True, point is in obstacle space


def setup_graph(ROBOT_RADIUS, CLEARANCE, BOARD_H, BOARD_W, GRID_D, point_robot=True):
    GRID_H = BOARD_H * GRID_D
    GRID_W = BOARD_W * GRID_D
    GRID_ROBOT_RADIUS = ROBOT_RADIUS * GRID_D
    GRID_CLEARANCE = CLEARANCE * GRID_D
    r = int(round(GRID_ROBOT_RADIUS + GRID_CLEARANCE))  # point robot radius

    obst = np.ones((GRID_H, GRID_W))

    # if no obstacles, then don't waste time looking for any
    if quads == [[]] and elips == [[]]:
        return obst

    # draw the regular obstacles
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

    if not point_robot:  # used to override the expansion for the visualization step
        return obst

    newObst = np.ones((GRID_H, GRID_W))  # create new obstacle array that will have r
    for x in range(GRID_W):
        for y in range(GRID_H):
            for i in range(x - r, x + r):  # window each pixel and check for an obstacle in radius
                for j in range(y - r, y + r):
                    if 0 <= i < GRID_W and 0 <= j < GRID_H:  # makes sure point is within bounds
                        if obst[j, i] == 0 and sqrt((x - i) ** 2 + (y - j) ** 2) < r:  # if window point is in obstacle
                            newObst[GRID_H - y, x] = 0
                            break
                else:
                    continue
                break

    return newObst
