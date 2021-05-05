#!/usr/bin/env python

import numpy as np
from cv2 import circle, line
import rospy
from math import sqrt, cos, sin, pi as PI
from random import sample, shuffle, seed
from terple_msgs.msg import NeighborsPose2D, BacktrackNode, MoveCommand, Path, AllPaths
# from geometry_msgs.msg import Pose2D, Twist, Pose, Point, Quaternion
from geometry_msgs.msg import Pose2D
from sys import stdout
import matplotlib.pyplot as plt
from copy import copy


ROS_NODE_NAME = "centralized_manager"


def printr(text):
    stdout.write("\r" + text)
    stdout.flush()


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
        if self.link_parent is not None:
            self.link_parent.link_child = self.link_child
        if self.link_child is not None:
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
    def __init__(self, obst):
        self.obst = obst  # creates the obstacle space. 0 for obstacle, 1 for open space [y, x, t]
        self.wheel_speeds = (
            int(WHEEL_SPEED_MINOR * 0.75),
            WHEEL_SPEED_MINOR,
            WHEEL_SPEED_MAJOR,
            0
        )

    # Determine if a coordinate pair is in a traversable portion of the maze
    def is_in_board(self, j, i):
        sh = self.obst.space_time.shape
        return (j >= 0) and (i >= 0) and (j < sh[0]) and (i < sh[1]) and self.obst.in_board(j, i)

    # Calculate the distance between two points
    @staticmethod
    def dist(n1, n2):
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
        actual_start = (start[0], start[1], start[2] * BOARD_O)
        start_node.vertex_node.actual_position = actual_start
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
                Ul = self.wheel_speeds[Uln]
                Ur = self.wheel_speeds[Urn]
                actual_ii, actual_jj, actual_ori, dist_traveled, dt = curve(
                    ni,
                    nj,
                    orientation,
                    Ul,
                    Ur
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
                if not self.obst.is_valid(jj, ii, tt):
                    # print "not valid", (jj, ii, tt)
                    continue

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
                    neighbor_node.time = tt

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
            # printr("Planning{0}".format("." * (idx // 2000)))

        return final_node, nodes_visited


def make_plan(ii, ij, fi, fj, angle_to_center, obst):
    s = int(ij * GRID_D), int(ii * GRID_D), angle_to_center
    g = int(fj * GRID_D), int(fi * GRID_D), angle_to_center

    # create graph
    maze = Maze(obst)

    # search for goal with A*
    final_node, nodes_visited = maze.astar(s, g)

    # backtrack the search graph to get the path to goal
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
        # print n.time, n.parent.time
        n = n.parent

    #      success                   path
    return (final_node is not None), backtrack_path


class Obstacles(object):
    # An empty board
    empty = None

    # The initial bot positions
    start = None

    # The full space-time log
    space_time = None

    # A log of the points that the bots have been
    # Each entry in the points list is a list of all (y, x, t) that a bot_i has been
    points = None

    def __init__(self, starting_locs):
        self.empty = np.ones((GRID_H, GRID_W)).astype('uint8')
        self.space_time = np.ones((GRID_H, GRID_W, 1)).astype('uint8')
        self.startup(starting_locs)
        self.start = self.space_time[:, :, 0]
        self.points = []

    # Record an y,x,t point as an obstacle
    def mark(self, y, x, t):
        y, x, t = int(round(y)), int(round(x)), int(t)
        self.space_time[:, :, t] = circle(np.ascontiguousarray(self.space_time[:, :, t]), (x, y),
                                          TOTAL_GRID_CLEARANCE, 0, -1)

    # Mark the starting locations
    def startup(self, starting_locs):
        for p in starting_locs:
            self.mark(int(GRID_H - p[0] * GRID_D), int(p[1] * GRID_D), 0)

    # Check if a point is within an y,x,t obstacle
    def is_valid(self, y, x, t):
        y, x, t = int(round(y)), int(round(x)), int(t)
        # check if point is farther in future than obst already extends

        # if last stored is start, return True
        if self.space_time.shape[2] == 1:
            return True

        if t < 4:
            return True

        try:  # time layer exists
            if self.space_time[y, x, t] == 1:
                return True
            # rospy.sleep(0.1)
            return False
        except IndexError:  # time layer does not exist yet, use last time stored instead
            if self.space_time[y, x, -1] == 1:
                return True
            # rospy.sleep(0.1)
            return False

    # Check if time layer exists already
    def time_layer_exists(self, t):
        return self.space_time.shape[2] >= t + 1

    # Check if a point is within a static obstacle
    # Empty for now because there are no status obstacles
    @staticmethod
    def in_board(y, x):
        # y, x = int(round(y)), int(round(x))
        return True

    # Add layer to the top of space_time
    def join_layers(self, layer):
        layer = np.reshape(layer, (GRID_H, GRID_W, 1)).astype('uint8')
        self.space_time = np.concatenate((self.space_time, layer), axis=2).astype('uint8')

    # Take a path and update obst space-time to reflect moving bot
    def update_from_path(self, path):
        # setup for new path to be stored
        i = len(self.points)
        self.points.append([])

        # draw nodes on obst space_time
        for node in path:
            x = int(round(node.position.x))
            y = int(round(node.position.y))
            t = node.time

            # add the path to self.points
            self.points[i].append((y, x, t))

            # draw the point on space_time
            if not self.time_layer_exists(t):  # time layer doesnt exist yet
                self.join_layers(self.empty)  # create a new empty time layer
                for j in range(len(self.points)-1):  # fill it with the last known locs of the other bots
                    (yj, xj, tj) = self.points[j][-1]  # last known loc of bot j
                    self.mark(yj, xj, t)
                # time layer now exists and is populated with other bots locs
            self.mark(y, x, t)

        # propagate last known location of this bot up to highest time
        while len(self.points[-1]) < len(max(self.points, key=len)):  # this path is shorter than longest path so far
            (y, x, t) = self.points[-1][-1]  # the last know location for this bot
            t += 1
            self.points[-1].append((y, x, t))
            self.mark(y, x, t)

    def show(self):
        for t in range(0, len(self.points[-1]), 2):
            plt.imshow(self.space_time[:, :, t])
            plt.show()


def main():
    rospy.sleep(1)
    # set spawn parameters
    num_of_bots = 20  # could later be changed to be user input
    bot_spawn_radius = 4

    # get starting locations
    starting_locs = []  # [y, x] locations of bot spawns
    starting_angles = []  # angle from start to goal
    rad_btwn_bots = 2 * PI / num_of_bots
    for i in range(num_of_bots):  # arrange points in a circle
        starting_locs.append([
            BOARD_H / 2 + sin(rad_btwn_bots * i) * bot_spawn_radius,
            BOARD_W / 2 + cos(rad_btwn_bots * i) * bot_spawn_radius
        ])
        # figure out the 0 - BOARD_O angle to get from start to goal
        a = i * rad_btwn_bots * 180 / PI
        a = int(a / BOARD_O)
        if a < GRID_O/2:
            a += GRID_O/2
        else:
            a -= GRID_O/2
        starting_angles.append(a)

    # rearrange list elements to get goal locations around the circle
    # make sure all bots need to move - ie. goal!=start for all bots
    seed(3)
    while True:
        goal_locs = sample(starting_locs, k=num_of_bots)
        for i in range(num_of_bots):
            if goal_locs[i] == starting_locs[i]:  # bot wouldn't move
                break
        else:
            break

    # print "\nstarting angles :", starting_angles
    # print "\nstarts :", starting_locs
    # print "goal   :", goal_locs, "\n"

    # initialize the obstacle grid
    obst = Obstacles(starting_locs)

    # plan paths for each robot in a random order
    bot_order = range(num_of_bots)
    shuffle(bot_order)
    all_paths_list = [None] * num_of_bots
    all_path_pub = rospy.Publisher(
        '/terple/all_paths',
        AllPaths,
        queue_size=1
    )
    path_msg = Path()
    max_path_length = 0
    for bot_i in bot_order:
        print "\nPlanning for Bot {0}".format(bot_i)

        # define start and goal points
        ii = starting_locs[bot_i][1]
        ij = starting_locs[bot_i][0]
        fi = goal_locs[bot_i][1]
        fj = goal_locs[bot_i][0]
        angle_to_center = starting_angles[bot_i]

        # Find the path
        success, backtrack = make_plan(ii, ij, fi, fj, angle_to_center, obst)

        # Check for failure
        if not success:
            print "Failed to find path for Bot {0}! Exiting...".format(bot_i)
            return
        else:
            print "Found Path for Bot {0} with Length {1} steps".format(bot_i, len(backtrack))

        # Convert to forward path, add to storage, update obst
        backtrack.reverse()
        path_msg.backtrack_path = backtrack
        all_paths_list[bot_i] = copy(path_msg)
        obst.update_from_path(backtrack)
        if len(backtrack) > max_path_length:
            max_path_length = len(backtrack)

        # print obst.points[-1]
        # obst.show()

    print "\nFinished Planning. Publishing Paths..."
    all_path_pub.publish(bot_paths=all_paths_list, max_path_length=max_path_length)


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
    GRID_CLEARANCE = CLEARANCE * GRID_D
    TOTAL_GRID_CLEARANCE = int(round(2*GRID_ROBOT_RADIUS + GRID_CLEARANCE))  # point robot radius

    main()

# # draw the initial robot circles on grid t=0
# obst_0 = np.copy(obst_init)
# for p in starting_locs:
#     obst_0 = circle(obst_0, (int(p[1] * GRID_D), int(GRID_H - p[0] * GRID_D)), TOTAL_GRID_CLEARANCE, 0, -1)
#     obst_0 = np.reshape(obst_0, (GRID_H, GRID_W, 1)).astype('uint8')
# obst = np.concatenate((obst, obst_0), axis=2).astype('uint8')
