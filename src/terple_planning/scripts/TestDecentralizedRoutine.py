#!/usr/bin/env python

import numpy as np
import cv2
from sys import argv as sargv
from terple_msgs.msg import Vector2, DecentralizedRobotReadings, RobotExternalCharacteristics

import DecentralizedRobotPlanner

# Number of pixels on both the y and x axes
MY_CV_SIZE = 800
MY_CV_HALF_SIZE = MY_CV_SIZE//2
MY_CV_ORIGIN = (MY_CV_HALF_SIZE,MY_CV_HALF_SIZE)
# Cartesian length of one quadrant for both the x and y axes
MY_CV_POS_SPACE = 3.0
MY_CV_VEL_SPACE = 2.0
# Pixels/meter, accounting for all four quadrants
MY_CV_POS_SCALE = MY_CV_SIZE / (2*MY_CV_POS_SPACE)
MY_CV_VEL_SCALE = MY_CV_SIZE / (2*MY_CV_VEL_SPACE)

def white_image_with_axes(shape):
    img = np.ones(shape, dtype=np.uint8)*255
    img = cv2.line(img, (MY_CV_HALF_SIZE,0), (MY_CV_HALF_SIZE,MY_CV_SIZE), (0,0,0), 2)
    img = cv2.line(img, (0,MY_CV_HALF_SIZE), (MY_CV_SIZE,MY_CV_HALF_SIZE), (0,0,0), 2)
    return img

def scale(v,s):
    nx = int(v.x*s)+MY_CV_HALF_SIZE
    ny = MY_CV_HALF_SIZE-int(v.y*s)
    return nx, ny

def inv_scale(nx,ny,s):
    vx = (nx-MY_CV_HALF_SIZE) / s
    vy = (MY_CV_HALF_SIZE-ny) / s
    return Vector2(x=vx,y=vy)

def scale_pos(v):
    return scale(v, MY_CV_POS_SCALE)

def inv_scale_pos(nx,ny):
    return inv_scale(nx, ny, MY_CV_POS_SCALE)

def scale_vel(v):
    return scale(v, MY_CV_VEL_SCALE)

def inv_scale_vel(nx,ny):
    return inv_scale(nx, ny, MY_CV_VEL_SCALE)

def scale_radius(r):
    return int(r*MY_CV_POS_SCALE)

def main(vmax, tau, rA, pA, vA, other_terples):
    # Constant colors
    COLORA = (0,0,255)
    COLORB = (255,0,0)

    # Calculate once and done
    rA_scaled = scale_radius(rA)
    vel_far_x = MY_CV_VEL_SPACE * 4

    # Draw the position-space image
    img_pos = white_image_with_axes((MY_CV_SIZE,MY_CV_SIZE,3))
    img_pos = cv2.circle(img_pos, scale_pos(pA), rA_scaled, COLORA, 2)
    for rb in other_terples:
        img_pos = cv2.circle(img_pos, scale_pos(Vector2(x=rb[1],y=rb[2])), scale_radius(rb[0]), COLORB, 2)

    imgs_voab = []
    for other_terple in other_terples:
        # Get the first robot in the list to be robot B
        rB = other_terple[0]
        pB = Vector2(x=other_terple[1],y=other_terple[2])
        vB = Vector2(x=other_terple[3],y=other_terple[4])

        # Create the test velocity obstacle
        voab = DecentralizedRobotPlanner.VelocityObstacleModel(rA, rB, pA, pB, tau)

        # Do misc calculations once and done
        vo_diff = DecentralizedRobotPlanner.vec2_diff(vA, vB)
        r_circ = int(voab.circle_trunc.r*MY_CV_VEL_SCALE)

        # Calculate points (interpolated, for the case of the legs) to test the calculated lines
        right_x = voab.points[1].x
        line_leg1, line_leg2, line_cross = voab.lines
        line_pts = [scale_vel(l.eval_at(x)) for x,l in zip(
            [-vel_far_x, -vel_far_x, vel_far_x, vel_far_x, right_x],
            [line_leg1, line_leg2, line_leg1, line_leg2, line_cross]
        )]

        # Draw the velocity-space image
        img_voab = white_image_with_axes((MY_CV_SIZE,MY_CV_SIZE,3))
        # Shade in the velocity obstacle
        for j in range(MY_CV_SIZE):
            for i in range(MY_CV_SIZE):
                if voab.contains(inv_scale_vel(i,j)):
                    img_voab[j,i] = (192,192,192)
        # Draw the robots' velocities
        img_voab = cv2.circle(img_voab, scale_vel(vA), rA_scaled, COLORA, 2)
        img_voab = cv2.circle(img_voab, scale_vel(vB), scale_radius(rB), COLORB, 2)
        # Draw the circle that truncates the cone
        img_voab = cv2.line(img_voab, line_pts[0], line_pts[2], (0,255,0), 2)
        img_voab = cv2.line(img_voab, line_pts[1], line_pts[3], (0,255,0), 2)
        img_voab = cv2.line(img_voab, scale_vel(voab.points[0]), line_pts[4], (0,255,0), 2)
        img_voab = cv2.circle(img_voab, scale_vel(voab.circle_trunc.position), 5, (128,0,128), -1)
        img_voab = cv2.circle(img_voab, scale_vel(voab.circle_trunc.position), r_circ, (128,0,128), 1)
        img_voab = cv2.circle(img_voab, scale_vel(voab.points[0]), 5, (255,255,0), -1)
        img_voab = cv2.circle(img_voab, scale_vel(voab.points[1]), 5, (0,165,255), -1)
        # Draw a velocity vA-vB in velocity-space and the closest boundary point if it's in the velocity obstacle
        vo_diff_color = None
        closest_to_boundary = None
        if voab.contains(vo_diff):
            vo_diff_color = (0,0,192)
            vec_u, closest_point, closest_dist = voab.boundary_point_closest_to(vo_diff)
            img_voab = cv2.line(img_voab, scale_vel(vo_diff), scale_vel(closest_point), (193,140,255), 2)
            img_voab = cv2.circle(img_voab, scale_vel(closest_point), 5, (193,140,255), -1)
        else:
            vo_diff_color = (0,128,0)
        img_voab = cv2.circle(img_voab, scale_vel(vo_diff), 5, vo_diff_color, -1)

        # Finished
        imgs_voab.append(img_voab)

    # Create a Terplebot
    terpleA = DecentralizedRobotPlanner.Terplebot(1, rA, vmax)
    terpleA.own_ext_char.position = pA
    terpleA.own_ext_char.velocity = vA
    terpleA.sensor_readings = DecentralizedRobotReadings(data=[
        RobotExternalCharacteristics(
            robot_id=i+2,
            position=Vector2(x=terp[1],y=terp[2]),
            velocity=Vector2(x=terp[3],y=terp[4]),
            radius=terp[0]
        ) for i,terp in enumerate(other_terples)
    ])
    terpleA.initialized = True

    # Build ORCA for robot A
    orcaA = DecentralizedRobotPlanner.build_ORCA_A_tau_for(terpleA, tau)
    # Draw the ORCA image for robot A
    img_orca = white_image_with_axes((MY_CV_SIZE,MY_CV_SIZE,3))
    # Shade in the acceptable velocities
    for j in range(MY_CV_SIZE):
        for i in range(MY_CV_SIZE):
            if orcaA.contains(inv_scale_vel(i,j)):
                img_orca[j,i] = (192,192,192)
    # Draw the half-plane equations
    for leq,toggle in orcaA.orca_tau_AXs:
        img_orca = cv2.line(
            img_orca,
            scale_vel(leq.eval_at(-vel_far_x)),
            scale_vel(leq.eval_at(vel_far_x)),
            np.random.randint(255, size=3),
            2
        )
    # Draw the optimal velocity of robot A
    vA_opt = DecentralizedRobotPlanner.get_opt_vel(vA)
    img_orca = cv2.circle(
        img_orca,
        scale_vel(vA_opt),
        10,
        (0,0,255),
        -1
    )
    # Draw the new velocity of robot A
    v_new_valid, v_new = orcaA.calculate_next_velocity(vA_opt)
    if v_new_valid:
        img_orca = cv2.circle(
            img_orca,
            scale_vel(v_new),
            6,
            (0,255,0),
            -1
        )

    cv2.imshow("Position Space", img_pos)
    for i in range(len(imgs_voab)):
        cv2.imshow("Velocity Space (B{0})".format(i), imgs_voab[i])
    cv2.imshow("ORCAA", img_orca)

    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    vmax = float(sargv[1])
    tau = float(sargv[2])
    rA, pAx, pAy, vAx, vAy = tuple(map(float, sargv[3].split(",")))
    other_terples = [tuple(map(float, arg.split(","))) for arg in sargv[4:]]
    print("A: {0},{1},{2},{3},{4}".format(rA, pAx, pAy, vAx, vAy))
    print("The rest: {0}".format(other_terples))

    main(
        tau,
        vmax,
        rA,
        Vector2(x=pAx,y=pAy),
        Vector2(x=vAx,y=vAy),
        other_terples
    )
