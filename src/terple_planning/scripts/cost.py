from math import sqrt, cos, sin, pi as PI
import rospy


def cost(xn, yn, theta_deg, ul, ur, dt=0.1, tf=1):
    r = rospy.get_param("/terple/robot_description/r")
    L = rospy.get_param("/terple/robot_description/L")
    GRID_D = rospy.get_param("/terple/space_description/GRID_D")

    # reformat to be in 10x10 grid system for calculations
    xn /= float(GRID_D)
    yn /= float(GRID_D)

    theta_rad = PI * theta_deg / 180.0  # convert deg to rad

    # draw 10 shorter lines to approximate the curve
    t = 0
    dist_traveled = 0
    while t < tf - dt/2.0:
        t = t + dt  # step the time by increment dt
        xs = xn  # start point x - set as previous goal
        ys = yn  # start point y - set as previous goal

        dd = 0.5 * r * (ul + ur) * dt
        dxn = dd * cos(theta_rad)
        dyn = dd * sin(theta_rad)
        xn += dxn
        yn += dyn
        theta_rad += ((r / L) * (ur - ul) * dt)
        dist_traveled += sqrt(dxn**2 + dyn**2)

    theta_deg = 180 * theta_rad / PI
    xn *= GRID_D
    yn *= GRID_D
    dist_traveled *= GRID_D
    return xn, yn, theta_deg, dist_traveled


