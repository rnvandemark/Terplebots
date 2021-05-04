#!/usr/bin/env python

import rospy
from terple_msgs.msg import RobotExternalCharacteristics, DecentralizedRobotReadings, Vector2
from threading import Lock
from math import sqrt, sin, cos, atan2
from sys import argv as sargv

ROS_NODE_NAME = "terple_decentralized_terplebot"

ORIGIN = Vector2(x=0,y=0)

# Helper function to safely get a rosparam
def get_rosparam(name):
    value = None
    if rospy.has_param(name):
        value = rospy.get_param(name)
    return value

# Helper function to print carriage return
def printr(text):
    stdout.write("\r" + text)
    stdout.flush()

# Helper function to get the slope and y-intercept of a line given two terple_msgs.Vector2 points
def get_line_from_points(v1, v2):
    dx = v2.x-v1.x
    if dx == 0:
        return None,None
    m = (v2.y-v1.y)/dx
    b = v2.y-(m*v2.x)
    return m,b

# Helper function to get the slope and y-intercept of a line given a slope and terple_msgs.Vector2 point
def get_line_from_slope_and_point(m, v):
    b = v.y - (m*v.x)
    return m,b

# Helper function to get the tranalte a terple_msgs.Vector2 element by (dx,dy)
def vec2_translate(v, dx, dy):
    return Vector2(
        x=v.x+dx,
        y=v.y+dy
    )

# Helper function to get the sum of two terple_msgs.Vector2 elements
def vec2_sum(v2, v1):
    return vec2_translate(v2, v1.x, v1.y)

# Helper function to get the difference of two terple_msgs.Vector2 elements
def vec2_diff(v2, v1):
    return vec2_translate(v2, -v1.x, -v1.y)

# Helper function to get a terple_msgs.Vector2 scaled by the reciprocal of some constant
def vec2_div(v, c):
    return Vector2(
        x=v.x/c,
        y=v.y/c
    )

# Helper function to get quadrant number given a 2D vector
def quadrant(v):
    if v.x >= 0:
        if v.y >= 0:
            return 0
        else:
            return 3
    else:
        if v.y >= 0:
            return 1
        else:
            return 2

# Helper function to query if a point is in the acceptable portion of a half-plane
def half_plane_contains(line, vec, toggle, is_open):
    a = (line[0]*vec.x) + line[1] - vec.y
    if toggle:
        return a > 0 if is_open else a >= 0
    else:
        return a < 0 if is_open else a <= 0

# Helper function to get a point on the given line that is closest to the given point
def point_on_line_closest_to(line, vec):
    x0, y0, a, c = vec.x, vec.y, -line[0], -line[1]
    denom = a**2 + 1
    x = (x0 - (a*y0) - (a*c)) / denom
    y = (a * (-x0 + (a*y0)) - c) / denom
    return Vector2(x=x,y=y), Vector2(x=x-x0,y=y-y0)

# Given the current velocity, calculate the optimal velocity
def get_opt_vel(curr_vel):
    return curr_vel
    #return ORIGIN

# A semi-algebraic model for a circle/disk
class CircleSemiAlgebraicModel(object):

    # The terple_msgs.Vector2 position of the center of the circle
    position = None

    # The radius of the circle
    r = None

    # Capture the y,x position and radius
    def __init__(self, position, r):
        self.position = position
        self.r = r

    # Check if this model contains the given coordinate
    def contains(self, coord, is_open):
        dist = (coord.x-self.position.x)**2 + (coord.y-self.position.y)**2
        if is_open:
            return dist < self.r**2
        else:
            return dist <= self.r**2

# A representation of a single, orphaned terplebot
class Terplebot(object):

    # Mutex to access the characteristics in this object safely across threads
    mutex = None

    # Whether or not the initial characteristics have been set
    initialized = None

    # The maximum velocity of this robot
    max_vel = None

    # The external characteristics of this robot
    own_ext_char = None

    # A collection of the external characteristics of all robots, including this one
    sensor_readings = None

    # The destination waypoint
    objective = None

    # Constructor
    def __init__(self, robot_id, radius, max_vel):
        self.mutex = Lock()
        self.initialized = False
        self.max_vel = max_vel
        self.own_ext_char = RobotExternalCharacteristics(robot_id=robot_id, position=None, velocity=None, radius=radius)
        self.sensor_readings = None

    # Update the 'sensor readings' that this robot has of the other robots' external characteristics
    def set_sensor_readings(self, readings):
        self.mutex.acquire()
        try:
            self.sensor_readings = readings
        finally:
            self.mutex.release()

# A model for a velocity obstacle
class VelocityObstacleModel(object):

    # A circle semi-algebraic model representing the truncated portion of the velocity obstacle
    circle_trunc = None

    # A 2-tuple of terple_msgs.Vector2 elements, representing the two points used to build the
    # 'legs' of the truncated cone from the origin of the velocity space 
    points = None

    # A 3-tuple of (m,b) pairs that represents the three lines used to make the cone-shaped portion
    # of the velocity obstacle's representation in velocity space
    lines = None

    # Construct model of polygon
    def __init__(self, rA, rB, pA, pB, tau):
        # Create a semi-algebraic model for the circle that truncates the velocity obstacle
        circle_trunc_position = vec2_div(vec2_diff(pB, pA), tau)
        circle_trunc_radius = (rA+rB)/tau
        self.circle_trunc = CircleSemiAlgebraicModel(circle_trunc_position, circle_trunc_radius)

        # Get the angle off the x-axis of a line with the slope that is tangential to the line
        # that passes through the origin and the center of the circle
        angle_tangent_to_boundary = atan2(-circle_trunc_position.x, circle_trunc_position.y)
        # Calculate the points on either side of the circle
        pts_dx = circle_trunc_radius * cos(angle_tangent_to_boundary)
        pts_dy = circle_trunc_radius * sin(angle_tangent_to_boundary)
        self.points = (
            vec2_translate(circle_trunc_position, -pts_dx, -pts_dy),
            vec2_translate(circle_trunc_position, pts_dx, pts_dy)
        )

        # Build three lines to build the half-plane model
        self.lines = (
            get_line_from_points(ORIGIN, self.points[0]),
            get_line_from_points(ORIGIN, self.points[1]),
            get_line_from_points(self.points[0], self.points[1])
        )

    # Check that a velocity vector is within the bounds of this model
    def contains(self, v):
        qr = quadrant(self.circle_trunc.position)
        in_legs = (
            half_plane_contains(self.lines[0], v, (self.lines[0][0] > 0) ^ ((qr == 2) or (qr == 3)), False) and
            half_plane_contains(self.lines[1], v, (self.lines[1][0] < 0) ^ ((qr == 2) or (qr == 3)), False)
        )
        if self.circle_trunc.contains(v, False):
            return in_legs
        else:
            return in_legs and half_plane_contains(self.lines[2], v, (self.lines[2][0] > 0) ^ ((qr == 1) or (qr == 2)), False)

    # Find the point on the boundary of the velocity obstacle that is closest to a point inside the boundary
    # Returns the the coordinate on the boundary as well as the distance
    # Assumes the given vector is within the bounds of the obstacle
    def boundary_point_closest_to(self, v):
        # Get the closest point on the top semi-circle, if it's in that area
        point_circle = None
        u_circle = None
        dist_circle = None
        qr = quadrant(self.circle_trunc.position)
        if half_plane_contains(self.lines[2], v, not ((self.lines[2][0] > 0) ^ ((qr == 1) or (qr == 2))), False):
            angle = atan2(v.y - self.circle_trunc.position.y, v.x - self.circle_trunc.position.x)
            point_circle = vec2_translate(
                self.circle_trunc.position,
                self.circle_trunc.r*cos(angle),
                self.circle_trunc.r*sin(angle)
            )
            u_circle = vec2_diff(point_circle, v)
            dist_circle = sqrt(u_circle.x**2 + u_circle.y**2)

        # Get the closest point to each leg of the cone
        point_leg1, u_leg1 = point_on_line_closest_to(self.lines[0], v)
        point_leg2, u_leg2 = point_on_line_closest_to(self.lines[1], v)
        dist_leg1 = sqrt(u_leg1.x**2 + u_leg1.y**2)
        dist_leg2 = sqrt(u_leg2.x**2 + u_leg2.y**2)

        # Now find the minimum distance and corresponding point
        min_dist = dist_leg1
        min_point = point_leg1
        min_u = u_leg1
        if dist_leg2 < min_dist:
            min_dist = dist_leg2
            min_point = point_leg2
            min_u = u_leg2
        if (dist_circle is not None) and (dist_circle < min_dist):
            min_dist = dist_circle
            min_point = point_circle
            min_u = u_circle

        return min_u, min_point, min_dist

# A model for allowable velocities of robot A given all other robots
class ORCAAModel(object):

    # A disk providing a bound given the max velocity of robot A
    max_disk = None

    # A list of 2-tuples, where the first element is a half-plane equation and the second is
    # a boolean as to whether or not to toggle its region
    orca_tau_AXs = None

    # Construct the max bounds and initialize an empty list for ORCAs
    def __init__(self, v_max):
        self.max_disk = CircleSemiAlgebraicModel(ORIGIN, v_max)
        self.orca_tau_AXs = []

    # Append a half-plane equation due to 
    def append_orca_tau_AX(self, line, toggle):
        self.orca_tau_AXs.append((line, toggle))

    # Check that a velocity vector is within the bounds of this model
    def contains(self, v, i=None):
        return self.max_disk.contains(v, False) and all(half_plane_contains(l, v, t, False) for l,t in self.orca_tau_AXs[:i])

    # Use linear programming to iterate through the half-plane constraints to find the velocity
    # closest to the given preferred one
    def calculate_next_velocity(self, v_pref, dx=0.01):
        # Initialize the new velocity as the preferred one
        is_valid = True
        v_new = v_pref
        assert(self.max_disk.contains(v_new, False))
        # Loop through constraints, updating the best option as v_new
        for i,(line,toggle) in enumerate(self.orca_tau_AXs):
            if not half_plane_contains(line, v_new, toggle, False):
                v_original, _ = point_on_line_closest_to(line, v_new)
                if self.contains(v_original, i=i+1):
                    v_new = v_original
                else:
                    v_temp = v_original
                    fdx = dx if line[0] > 0 else -dx
                    while True:
                        x = v_temp.x + fdx
                        v_temp = Vector2(x=x,y=(line[0]*x)+line[1])
                        if not self.max_disk.contains(v_temp, False):
                            is_valid = False
                            break
                        elif self.contains(v_temp, i=i+1):
                            v_new = v_temp
                            break
                    if not is_valid:
                        break
        return is_valid, v_new

# Build the velocity obstacle of A induced by B for time window tau
# terplebot: the bot to identify as robot A
# tau: the time window to hopefully guarantee no collisions for
def build_ORCA_A_tau_for(terplebot, tau):
    orcaA = None
    terplebot.mutex.acquire()
    try:
        if terplebot.initialized:
            rA = terplebot.own_ext_char.radius
            pA = terplebot.own_ext_char.position
            vA_opt = get_opt_vel(terplebot.own_ext_char.velocity)
            orcaA = ORCAAModel(terplebot.max_vel)
            for other_ext_char in terplebot.sensor_readings.data:
                if terplebot.own_ext_char.robot_id == other_ext_char.robot_id:
                    continue
                voAB = VelocityObstacleModel(rA, other_ext_char.radius, pA, other_ext_char.position, tau)
                dv_opt = vec2_diff(vA_opt, get_opt_vel(other_ext_char.velocity))
                if voAB.contains(dv_opt):
                    u_to_boundary, _, _ = voAB.boundary_point_closest_to(dv_opt)
                    half_plane_point = vec2_sum(vA_opt, vec2_div(u_to_boundary, 2))
                    half_plane_equation = get_line_from_slope_and_point(
                        -u_to_boundary.x / u_to_boundary.y,
                        half_plane_point
                    )
                    orcaA.append_orca_tau_AX(half_plane_equation, False)
    finally:
        terplebot.mutex.release()
    return orcaA

# Set a bot's readings of external characteristics
def robot_readings_callback(msg, terplebot):
    terplebot.set_sensor_readings(msg)

def main():
    # Capture required user input
    my_sargv = rospy.myargv(argv=sargv)
    assert(len(my_sargv) == 2)

    robot_id = None
    try:
        robot_id = int(my_sargv[1])
    except:
        print "Please enter the robot ID as an integer."
        return

    # Init ROS elements and get parameters
    rospy.init_node(ROS_NODE_NAME)
    tau_seconds_theoretical = get_rosparam("/terple/decentralized/tau_seconds_theoretical")
    tau_seconds_actual = get_rosparam("/terple/decentralized/tau_seconds_actual")
    robot_radius = get_rosparam("/terple/decentralized/robot_radius")
    robot_max_vel = get_rosparam("/terple/decentralized/max_vel")
    assert(
        (tau_seconds_theoretical != None) and
        (tau_seconds_actual != None) and
        (robot_radius != None) and
        (robot_max_vel != None)
    )

    # Create collection of robot external characteristics
    terplebot = Terplebot(robot_id, robot_radius, robot_max_vel)

    # Init ROS pub and sub
    robot_readings_sub = rospy.Subscriber(
        "/terple/decentralized/robot_readings",
        DecentralizedRobotReadings,
        lambda m: robot_readings_callback(m, terplebot),
        queue_size=1
    )
    characteristics_updates_pub = rospy.Publisher(
        "/terple/decentralized/characteristics_updates",
        RobotExternalCharacteristics,
        queue_size=1
    )

    # Start spin routine
    ros_duration = rospy.Duration(tau_seconds_actual)
    while not rospy.is_shutdown():
        # The robot is accepting readings in another threads whenever they are made available
        terplebot.mutex.acquire()
        try:
            if terplebot.initialized:
                self.sensor_readings = readings
        finally:
            terplebot.mutex.release()

        # Sleep for tau actual
        ros_duration.sleep()

if __name__ == "__main__":
    main()
