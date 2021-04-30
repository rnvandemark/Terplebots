#!/usr/bin/env python

from math import sqrt, cos, sin, pi as PI
import rospy
from cv2 import line


def curve(xn, yn, theta_deg, ul, ur, dt=0.1, tf=1, img=None, scl=None):
    # img is an option arg. if present then func will also draw the curve on the img
    # Must also pass scl with image. represents how much bigger the output viz video is than the grid

    r = rospy.get_param("/terple/robot_description/r")
    L = rospy.get_param("/terple/robot_description/L")
    GRID_D = rospy.get_param("/terple/space_description/GRID_D")
    GRID_H = rospy.get_param("/terple/space_description/GRID_H")

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
        dd = 0.5 * r * (ul + ur) * dt
        dxn = dd * cos(theta_rad)
        dyn = dd * sin(theta_rad)
        xn += dxn
        yn += dyn
        theta_rad += ((r / L) * (ur - ul) * dt)
        dist_traveled += sqrt(dxn ** 2 + dyn ** 2)

        if img:
            img = line(img, (int(xs * scl * GRID_D), int(GRID_H * scl - ys * scl * GRID_D)),
                       (int(xn * scl * GRID_D), int(GRID_H * scl - yn * scl * GRID_D)),
                       (0, 0, 255))  # draw the small line, with scaled points

    # convert back to working dimensions
    theta_deg = 180 * theta_rad / PI
    xn *= GRID_D
    yn *= GRID_D
    dist_traveled *= GRID_D

    # return img only if one is given
    if img:
        return xn, yn, theta_deg, dist_traveled, img
    return xn, yn, theta_deg, dist_traveled
