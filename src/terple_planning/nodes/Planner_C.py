#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D
from terple_msgs.msg import PlanRequest, NeighborsPose2D, BacktrackNode, MoveCommand, Path
from math import sqrt, cos, sin, pi as PI
from sys import stdout, argv as sargv
from gc import collect as gc_collect
from terple_planning.scripts import obstacles, cost

ROS_NODE_NAME = "centralized_planner"

BOARD_H = 10
BOARD_W = 10
BOARD_O = 30
ROBOT_RADIUS = 0.153

GRID_D = 10
GRID_H = BOARD_H * GRID_D
GRID_W = BOARD_W * GRID_D
GRID_O = int(360 / BOARD_O)
GRID_ROBOT_RADIUS = ROBOT_RADIUS * GRID_D


def get_rosparam(name):
    value = None
    if rospy.has_param(name):
        value = rospy.get_param(name)
    return value


def printr(text):
    stdout.write("\r" + text)
    stdout.flush()


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

    # Constructor, given all values
    def __init__(self, parent, position, distG, distF):
        self.parent = parent
        self.position = position
        self.distG = distG
        self.distF = distF
        self.twist_elements_to_here = (0, 0, 0)
        self.actual_position = None


# A link in a priorty queue chain
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


# A collection of a couple of physical characteristics of the robot
class RobotDescription(object):
    # The wheel speed multipliers
    wheel_speeds = None

    # The wheel radius
    r = None

    # The lateral distance between the wheels
    L = None

    # Collect data about the robot
    def __init__(self, r, L):
        self.wheel_speeds = (0, 0, 0)
        self.r = robot_r
        self.L = robot_L


# A pathfinding object which builds a representation of the underlying maze
class Maze(object):
    # The DiscreteGraph representation of the maze
    obst = None

    # The wheel speed multiplier
    robo_desc = None

    # Build the graph with the list of semi-algebraic models
    def __init__(self, clearance, r, L):
        self.obst = obstacles.setup_graph()  # creates the obstacle space. 0 for obstacle, 1 for open space
        self.robo_desc = RobotDescription(r, L)

    # Determine if a coordinate pair is in a traversable portion of the maze
    def is_in_board(self, j, i):
        sh = self.obst.shape
        return (j >= 0) and (i >= 0) and (j < sh[0]) and (i < sh[1]) and (self.obst[j, i] == 1)

    # Calculate the distance between two points
    def dist(self, n1, n2):
        return sqrt((n2[1] - n1[1]) ** 2 + (n2[0] - n1[0]) ** 2)

    # Calculate the tentative remaining distance from n to goal, given this heuristic
    def h(self, n, goal):
        return self.dist(n, goal)

    # Run A* between a start and goal point, using a forward step length
    def astar(self, start, goal, minor_wheel_speed, major_wheel_speed):
        self.robo_desc.wheel_speeds = (
            int(minor_wheel_speed * 0.75),
            minor_wheel_speed,
            major_wheel_speed,
            0
        )

        all_maze_vertex_nodes_map = {}
        for j in range(GRID_H):
            for i in range(GRID_W):
                if self.is_in_board(j, i):
                    for o in range(GRID_O):
                        v = (j, i, o);
                        a = None;
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
                actual_ii, actual_jj, actual_ori, dist_traveled, dt = cost.curve(
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

                neighbor_node = all_maze_vertex_nodes_map.get(neighbor_position, None)
                if neighbor_node is None:
                    # This position is not an acceptable position
                    # Either it's been visited, it's outside the bounds, or it's in an obstacle
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
                moves_to_neighbors=moves_to_neighbors
            ))
            idx = idx + 1
            printr("Planning{0}".format("." * (idx // 2000)))

        print
        return final_node, nodes_visited


def handle_plan_request(msg, maze, pub):
    path_msg = Path()
    path_msg.success = False
    path_msg.request = msg

    s = int(msg.init_position.y * GRID_D), int(msg.init_position.x * GRID_D), 0
    g = int(msg.final_position.y * GRID_D), int(msg.final_position.x * GRID_D), 0

    print "Running A* with {0} => {1}...".format(s, g)
    final_node, nodes_visited = maze.astar(s, g, msg.wheel_speed_minor, msg.wheel_speed_major)
    print "Finished, visited {0}".format(len(nodes_visited))

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
                time_elapsed=n.twist_elements_to_here[2]
            )
        ))
        n = n.parent

    path_msg.success = (final_node is not None)
    path_msg.explored = nodes_visited
    path_msg.backtrack_path = backtrack_path

    pub.publish(path_msg)


def cleanly_handle_plan_request(msg, maze, pub):
    handle_plan_request(msg, maze, pub)
    gc_collect()


def main():
    # Build the maze and underlying graph object
    print "Starting maze generation..."
    maze = Maze()
    print "Done, waiting for path planning requests."

    # Init ROS pub and sub
    path_pub = rospy.Publisher("/terple/path", Path, queue_size=1)
    plan_request_sub = rospy.Subscriber(
        "/terple/plan_request",
        PlanRequest,
        lambda m: cleanly_handle_plan_request(m, maze, path_pub),
        queue_size=1
    )

    rospy.spin()


if __name__ == "__main__":
    # Init ROS elements and get parameters
    rospy.init_node(ROS_NODE_NAME)
    robot_r = get_rosparam("/terple/robot_description/r")
    robot_L = get_rosparam("/terple/robot_description/L")
    BOARD_H = rospy.get_param("/terple/space_description/BOARD_H")
    BOARD_W = rospy.get_param("/terple/space_description/BOARD_W")
    BOARD_O = rospy.get_param("/terple/space_description/BOARD_O")
    GRID_D = rospy.get_param("/terple/space_description/GRID_D")
    GRID_H = BOARD_H * GRID_D
    GRID_W = BOARD_W * GRID_D
    GRID_O = int(360 / BOARD_O)

    main()
