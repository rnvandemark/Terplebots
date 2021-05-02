#!/usr/bin/env python

import numpy as np
from cv2 import circle, line
import rospy
from math import sqrt, atan, cos, sin, pi as PI
from random import sample, shuffle
from terple_msgs.msg import PlanRequest, NeighborsPose2D, BacktrackNode, MoveCommand, Path
# from geometry_msgs.msg import Pose2D, Twist, Pose, Point, Quaternion
from geometry_msgs.msg import Pose2D


ROS_NODE_NAME = "centralized_manager"


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
    theta_deg = theta_deg % 360
    xn *= GRID_D
    yn *= GRID_D
    dist_traveled *= GRID_D

    # return img only if one is given
    if img:
        return xn, yn, theta_deg, dist_traveled, img
    return xn, yn, theta_deg, dist_traveled, tf


# A single state of the maze's search
class MazeVertexNode(object):
    # The wheel speeds that can be used to get to neighbors, in (L,R) pairs
    WHEEL_SPEEDS_TO_NEIGHBORS = (
        (0, 0),
        (0, 1),
        (0, 2),
        (1, 2),
        (1, 1),
        (2, 2),
        (1, 0),
        (2, 0),
        (2, 1),
        (3, 3)
    )

    # The parent MazeVertexNode to this instance
    parent = None

    # A 3-tuple, (y,x,t) coordinate pair, where t is an orientation theta
    position = None

    # The current tentative distance from the start to this node
    distG = None

    # The current tentative distance from this node to the goal, given some heuristic
    distF = None

    # The wheel speeds and time duration used to visit this node (theta_l, theta_r, dt)
    twist_elements_to_here = None

    # The actual position, non-truncated
    actual_position = None

    # The time from start that this node exists at
    time = None

    # Constructor, given all values
    def __init__(self, parent, position, distG, distF):
        self.parent = parent
        self.position = position
        self.distG = distG
        self.distF = distF
        self.twist_elements_to_here = (0, 0, 0)
        self.actual_position = None
        self.time = 0


# A link in a priority queue chain
class DoublyLinkNode(object):
    # An instance of MazeVertexNode
    vertex_node = None

    # The left / lower priority DoublyLinkNode
    link_parent = None

    # The right / higher priority DoublyLinkNode
    link_child = None

    # Special flag to ID if this is the left end of the chain
    is_left = None

    # Special flag to ID if this is the right end of the chain
    is_right = None

    # Create a link with a vertex node and parent
    def __init__(self, vertex_node):
        self.vertex_node = vertex_node
        self.link_parent = None
        self.link_child = None
        self.is_left = False
        self.is_right = False

    # Insert this node as a child to another
    # Assumes new_parent is not null
    def insert_as_child_to(self, new_parent):
        self.link_parent = new_parent
        self.link_child = new_parent.link_child
        if new_parent.link_child is not None:
            new_parent.link_child.link_parent = self
        new_parent.link_child = self

    # Helper function to remove this node from a chain it's in
    def remove_from_chain(self):
        if self.link_parent != None:
            self.link_parent.link_child = self.link_child
        if self.link_child != None:
            self.link_child.link_parent = self.link_parent
        self.link_parent = None
        self.link_child = None

    @staticmethod
    def get_left_inst():
        dln = DoublyLinkNode(None)
        dln.is_left = True
        return dln

    @staticmethod
    def get_right_inst():
        dln = DoublyLinkNode(None)
        dln.is_right = True
        return dln


# A pathfinding object which builds a representation of the underlying maze
class Maze(object):
    # The DiscreteGraph representation of the maze
    obst = None

    # The wheel speeds
    wheel_speeds = None

    # Build the graph with the list of semi-algebraic models
    def __init__(self, r, L, obstacles):
        self.obst = obstacles  # creates the obstacle space. 0 for obstacle, 1 for open space [y, x, t]
        self.wheel_speeds = (
            int(WHEEL_SPEED_MINOR * 0.75),
            WHEEL_SPEED_MINOR,
            WHEEL_SPEED_MAJOR,
            0
        )

    # Determine if a coordinate pair is in a traversable portion of the maze
    def is_in_board(self, j, i):
        sh = self.obst.shape
        return (j >= 0) and (i >= 0) and (j < sh[0]) and (i < sh[1]) and (self.obst[j, i, 0] == 1)

    # Calculate the distance between two points
    def dist(self, n1, n2):
        return sqrt((n2[1] - n1[1]) ** 2 + (n2[0] - n1[0]) ** 2)

    # Calculate the tentative remaining distance from n to goal, given this heuristic
    def h(self, n, goal):
        return self.dist(n, goal)

    # Run A* between a start and goal point, using a forward step length
    def astar(self, start, goal):

        all_maze_vertex_nodes_map = {}
        for j in range(GRID_H):
            for i in range(GRID_W):
                if self.is_in_board(j, i):
                    for o in range(GRID_O):
                        v = (j, i, o)
                        a = None
                        b = None
                        if v == start:
                            a = 0
                            b = self.h(start, goal)
                        else:
                            a = 999999999
                            b = 999999999
                        all_maze_vertex_nodes_map[v] = MazeVertexNode(None, v, a, b)

        # Create the left end of the chain, the right end, and the sole initial link as the start
        chain_left_node = DoublyLinkNode.get_left_inst()
        chain_right_node = DoublyLinkNode.get_right_inst()
        chain_right_node.insert_as_child_to(chain_left_node)
        start_node = DoublyLinkNode(all_maze_vertex_nodes_map[start])
        start_node.vertex_node.actual_position = start
        start_node.insert_as_child_to(chain_left_node)
        chain_links_map = {start: start_node}

        # Track the nodes that were visited in the order they were, to visualize later
        nodes_visited = []

        # Start the main part of the algorithm, tracking the node that can be used to recover the path
        final_node = None
        idx = 0
        while (final_node is None) and (len(chain_links_map) != 0):
            # Visit the node with the lowest fScore, which is highest in the queue
            visiting_link = chain_right_node.link_parent

            # Essentially, mark this node as "visited" and capture its position
            visiting_link.remove_from_chain()
            nP = visiting_link.vertex_node.position
            del chain_links_map[nP]
            del all_maze_vertex_nodes_map[nP]
            nP = visiting_link.vertex_node.actual_position
            tP = visiting_link.vertex_node.time

            # Check if this is the goal position
            if self.dist(nP, goal) <= 2:
                final_node = visiting_link.vertex_node
                continue

            # Track the moves used to explore the neighbors
            moves_to_neighbors = []

            # Get each of the neighbors of the node being visited by looping through the five possible actions
            nj, ni, orientation = nP
            for Uln, Urn in MazeVertexNode.WHEEL_SPEEDS_TO_NEIGHBORS:
                Ul = self.robo_desc.wheel_speeds[Uln]
                Ur = self.robo_desc.wheel_speeds[Urn]
                actual_ii, actual_jj, actual_ori, dist_traveled, dt = curve(
                    ni,
                    nj,
                    orientation,
                    Ul,
                    Ur,
                    r,
                    L,
                    GRID_D,
                    GRID_H
                )
                ii = int(actual_ii)
                jj = int(actual_jj)
                ori = int(actual_ori / BOARD_O)
                neighbor_position = (jj, ii, ori)
                tt = visiting_link.vertex_node.time + dt

                neighbor_node = all_maze_vertex_nodes_map.get(neighbor_position, None)
                if neighbor_node is None:
                    # This position is not an acceptable position
                    # Either it's been visited, it's outside the bounds, or it's in an obstacle
                    continue

                # check if neighbor_node conflicts with another bot in space-time obst space
                if tt > self.obst.shape[2]:  # time exceeds bounds, use last known pose of each bot
                    if self.obst[jj, ii, -1] == 0:
                        continue
                elif self.obst[jj, ii, tt] == 0:  # time is within bounds, check normally
                    continue

                # valid node in time, add time to neighbor_node
                neighbor_node.time = tt

                # Add the position of this neighbor to visualize later
                moves_to_neighbors.append(MoveCommand(
                    left_wheel_speed=Ul,
                    right_wheel_speed=Ur,
                    time_elapsed=dt
                ))

                # Calculate the adjusted distance
                node_distG = visiting_link.vertex_node.distG + dist_traveled
                if node_distG < neighbor_node.distG:
                    # Set this node as this neighbor's shortest path
                    neighbor_node.distG = node_distG
                    neighbor_node.distF = node_distG + self.h(neighbor_position, goal)
                    neighbor_node.parent = visiting_link.vertex_node
                    neighbor_node.twist_elements_to_here = (Ul, Ur, dt)
                    neighbor_node.actual_position = (actual_jj, actual_ii, actual_ori)

                    # Do a less costly sort by simply moving the neighbor node in the priority queue
                    if neighbor_position not in chain_links_map:
                        # Create a link in the chain and add it to the hashmap, to locate by position
                        neighbor_link = DoublyLinkNode(neighbor_node)
                        chain_links_map[neighbor_position] = neighbor_link
                        # Start the search from the right end / highest priority link in the chain
                        potential_new_link_parent = chain_right_node.link_parent
                        continue_prioritize = True
                        while continue_prioritize:
                            if potential_new_link_parent.is_left:
                                # The entire open set has been searched
                                neighbor_link.insert_as_child_to(chain_left_node)
                                continue_prioritize = False
                            elif potential_new_link_parent.vertex_node.distF >= neighbor_node.distF:
                                # Found the point in the chain where the parent has a higher
                                # fscore but child has a lower on
                                neighbor_link.insert_as_child_to(potential_new_link_parent)
                                continue_prioritize = False
                            else:
                                # Have not finished searching yet
                                potential_new_link_parent = potential_new_link_parent.link_parent
                            # raw_inPut("Press Enter to continue...")

            nodes_visited.append(NeighborsPose2D(
                position=Pose2D(x=ni, y=nj, theta=orientation),
                moves_to_neighbors=moves_to_neighbors,
                time=tP
            ))
            idx = idx + 1
            printr("Planning{0}".format("." * (idx // 2000)))

        print
        return final_node, nodes_visited


def init_plan(ii, ij, fi, fj, angle_to_center, obst):
    # path_msg = Path()
    # path_msg.success = False
    # path_msg.request = msg

    s = int(ij * GRID_D), int(ii * GRID_D), angle_to_center
    g = int(fj * GRID_D), int(fi * GRID_D), 0

    maze = Maze(robot_r, robot_L, obst)

    final_node, nodes_visited = maze.astar(s, g)

    backtrack_path = []
    n = final_node
    while n is not None:
        if n.actual_position is None:
            n = n.parent
            continue
        backtrack_path.append(BacktrackNode(
            position=Pose2D(x=n.actual_position[1], y=n.actual_position[0], theta=n.actual_position[2]),
            has_move_cmd=(n.parent is not None),
            move_cmd=MoveCommand(
                left_wheel_speed=n.twist_elements_to_here[0],
                right_wheel_speed=n.twist_elements_to_here[1],
                time_elapsed=n.twist_elements_to_here[2]),
            time=n.time
        ))
        n = n.parent

    # path_msg.success = (final_node is not None)
    # path_msg.explored = nodes_visited
    # path_msg.backtrack_path = backtrack_path
    #
    # pub.publish(path_msg)


def main():
    # set spawn parameters
    num_of_bots = 8  # could later be changed to be user input
    bot_spawn_radius = 4

    # get starting locations
    starting_locs = []  # [y, x] locations of bot spawns
    rad_btwn_bots = 2*PI/num_of_bots
    for i in range(num_of_bots):  # arrange points in a circle
        starting_locs.append([
            BOARD_H/2 + sin(rad_btwn_bots * i) * bot_spawn_radius,
            BOARD_W/2 + cos(rad_btwn_bots * i) * bot_spawn_radius
        ])

    # rearrange list elements to get goal locations around the circle
    goal_locs = sample(starting_locs, k=num_of_bots)

    # initialize the grid
    # obst_init = obstacles.setup_graph(ROBOT_RADIUS, CLEARANCE, BOARD_H, BOARD_W, GRID_D, starts=starting_locs)
    obst_init = np.ones((GRID_H, GRID_W)).astype('uint8')

    # prepare grid for time dimension
    obst = np.ones((GRID_H, GRID_W, 0)).astype('uint8')

    # draw the initial robot circles on grid t=0
    obst_0 = np.copy(obst_init)
    for p in starting_locs:
        obst_0 = circle(obst_0, (int(p[1] * GRID_D), int(GRID_H - p[0] * GRID_D)), TOTAL_GRID_CLEARANCE, 0, -1)
        obst_0 = np.reshape(obst_0, (GRID_H, GRID_W, 1)).astype('uint8')
    obst = np.concatenate((obst, obst_0), axis=2).astype('uint8')

    # for each robot
    #     publish a plan request to planner
    #     subscribe to planner to receive request  ->  spin_once
    #     handle plan received
    #         go through the backtrack path
    #         add locations to obst time
    #         add path to msg having list of paths for all bots

    # plan paths for each robot in a random order
    bot_order = range(num_of_bots)
    shuffle(bot_order)
    for bot_i in bot_order:
        rospy.sleep(0.5)
        # define start and goal points
        ii = starting_locs[bot_i][1]
        ij = starting_locs[bot_i][0]
        fi = goal_locs[bot_i][1]
        fj = goal_locs[bot_i][0]

        try:
            angle_to_center = atan((ij - BOARD_H/2)/(ii - BOARD_W/2))
            angle_to_center = angle_to_center * 180 / PI
        except ZeroDivisionError:
            if ij > BOARD_H/2:
                angle_to_center = -90
            else:
                angle_to_center = 90

        init_plan(ii, ij, fi, fj, angle_to_center, obst)


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
    GRID_H = BOARD_H * GRID_D
    GRID_W = BOARD_W * GRID_D
    GRID_O = int(360 / BOARD_O)
    GRID_ROBOT_RADIUS = ROBOT_RADIUS * GRID_D
    GRID_CLEARANCE = ROBOT_RADIUS * GRID_D
    TOTAL_GRID_CLEARANCE = int(round(GRID_ROBOT_RADIUS + GRID_CLEARANCE))  # point robot radius

    main()
