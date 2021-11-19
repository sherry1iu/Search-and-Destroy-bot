#!/usr/bin/env python

# Author: Elliot Potter, CS 81, Dartmouth College
# Date: 10/13/21

# import of relevant libraries.
import rospy  # module for ROS APIs
from geometry_msgs.msg import Twist  # message type for velocity command.
from nav_msgs.msg import OccupancyGrid

import math
import tf  # library for transformations.
import numpy

from a_star_search import AStarSearch

# Constants.
FREQUENCY = 10  # Hz.
VELOCITY = 0.2  # m/s

from std_msgs.msg import String

# the width and height of the desired clear square to search for
CLEAR_SQUARE_DIMENSIONS = 10


class GridPlanner:
    def __init__(self):
        """Initialization function."""
        # 2nd. setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.string_pub = rospy.Publisher("string", String, queue_size=1)

        self.subscriber = rospy.Subscriber("blur_map", OccupancyGrid, self.grid_callback)

        # start and goal positions will be retrieved from the command line
        self.start_pos = None
        self.goal_pos = None

        # this will be populated with the data from the grid
        self.grid_msg = None
        # this is the grid converted into a double-nested array
        self.grid_arrays = None

        # the corners of the targeted path
        self.target_corners_cache = []
        self.target_corners_queue = []

    def grid_callback(self, grid_msg):
        """The callback to set the grid instance variable"""
        self.grid_msg = grid_msg
        # converts grid_msg.data into a double-nested array
        self.grid_arrays = numpy.reshape(grid_msg.data, (grid_msg.info.height, grid_msg.info.width))
        # print(grid_msg.info)

    @staticmethod
    def get_pose(pos, yaw):
        """Get a Pose object based on x, y position and yaw"""

        # this is a little janky, but it's easiest to get a pose off of an instance of a class
        pose = Marker().pose

        pose.position.x = pos["x"]
        pose.position.y = pos["y"]
        # converts the yaw into a quaternion representation
        quaternion = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        return pose

    def find_route(self, initial_pos, goal_pos):
        """A wrapper function to call AStarSearch
        the function to find the route through the grid. Initial and final positions are in the
        grid frame of reference and adjusted based on the resolution of the grid
        """
        # we use A* search to find a route from the origin to the goal
        searcher = AStarSearch(initial_pos, goal_pos, self.grid_arrays)
        path = searcher.perform_search(75)
        return path

    # TODO -- look at the map and find a clear square at an extreme x and y
    def get_target_corners(self):
        # starting pos
        # extreme y pos
        # extreme x pos


    def get_positions(self):
        """Get initial and final positions from the command line"""

        initial_x = raw_input("x-coordinate of initial position: ")
        initial_y = raw_input("y-coordinate of initial position: ")
        goal_x = raw_input("x-coordinate of goal position: ")
        goal_y = raw_input("y-coordinate of goal position: ")

        self.start_pos = {"x": initial_x, "y": initial_y}
        self.goal_pos = {"x": goal_x, "y": goal_y}

    def plan(self):
        """The planning driver function"""
        print("Beginning planning")

        # gets the initial and final positions
        self.get_positions()

        # transforms the initial/final positions to the grid resolution
        grid_start_pos = {
            "x": int(float(self.start_pos["x"]) / self.grid_msg.info.resolution + self.grid_msg.info.origin.position.x),
            "y": int(float(self.start_pos["y"]) / self.grid_msg.info.resolution + self.grid_msg.info.origin.position.y),
        }
        grid_goal_pos = {
            "x": int(float(self.goal_pos["x"]) / self.grid_msg.info.resolution + self.grid_msg.info.origin.position.x),
            "y": int(float(self.goal_pos["y"]) / self.grid_msg.info.resolution + self.grid_msg.info.origin.position.y),
        }

        print("Finding the route using A* search")
        grid_path = self.find_route(grid_start_pos, grid_goal_pos)

        # scales the path back to the normal x and y lengths
        path = []
        for location in grid_path:
            path.append({
                "x": location["x"] * self.grid_msg.info.resolution,
                "y": location["y"] * self.grid_msg.info.resolution,
            })

        print(path)

        # publishes the pose messages and displays the arrows
        self.show_pose_sequence(path)

        # returns the path
        return path


if __name__ == "__main__":
    # 1st. initialization of node.
    rospy.init_node("planner")

    # 2nd. Creation of the class with relevant publishers/subscribers.
    t = GridPlanner()

    # let publishers and subscribers set up correctly
    rospy.sleep(2)

    # 3rd, begin the planning procedure.
    t.plan()

    rospy.spin()
