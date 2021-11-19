#!/usr/bin/env python

# Author: Elliot Potter, CS 81, Dartmouth College
# Date: 10/13/21

# import of relevant libraries.
import rospy  # module for ROS APIs
from geometry_msgs.msg import Twist  # message type for velocity command.
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped

import math
import tf  # library for transformations.
import numpy

from a_star_search import AStarSearch

# Constants.
FREQUENCY = 10  # Hz.
VELOCITY = 0.2  # m/s

from std_msgs.msg import String


class GridPlanner:
    def __init__(self):
        """Initialization function."""
        # 2nd. setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        self.string_pub = rospy.Publisher("string", String, queue_size=1)

        self.subscriber = rospy.Subscriber("map", OccupancyGrid, self.grid_callback)

        self.marker_pub = rospy.Publisher("markers", Marker, queue_size=1)

        self.pos_pub = rospy.Publisher("pose_sequence", PoseStamped, queue_size=1)

        # start and goal positions will be retrieved from the command line
        self.start_pos = None
        self.goal_pos = None

        # this will be populated with the data from the grid
        self.grid_msg = None
        # this is the grid converted into a double-nested array
        self.grid_arrays = None

    def grid_callback(self, grid_msg):
        """The callback to set the grid instance variable"""
        self.grid_msg = grid_msg
        # converts grid_msg.data into a double-nested array
        self.grid_arrays = numpy.reshape(grid_msg.data, (grid_msg.info.height, grid_msg.info.width))
        # print(grid_msg.info)

    def publish_marker(self, marker_id, pose):
        """
         the function used to publish a marker -- copied from example code and modified slightly
        """
        marker_msg = Marker()
        marker_msg.header.stamp = rospy.Time.now()
        marker_msg.header.frame_id = "map"
        marker_msg.action = Marker.ADD
        marker_msg.type = Marker.ARROW
        marker_msg.id = marker_id
        marker_msg.pose = pose
        marker_msg.color.r = 1.0
        marker_msg.color.a = 1
        marker_msg.scale.x = 1
        marker_msg.scale.y = 0.1
        marker_msg.scale.z = 0.1
        # publish the marker

        self.marker_pub.publish(marker_msg)

    def publish_pose_stamped(self, pose):
        """Publish a PoseStamped message"""
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = pose
        self.pos_pub.publish(pose_stamped)

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

    def get_positions(self):
        """Get initial and final positions from the command line"""

        initial_x = raw_input("x-coordinate of initial position: ")
        initial_y = raw_input("y-coordinate of initial position: ")
        goal_x = raw_input("x-coordinate of goal position: ")
        goal_y = raw_input("y-coordinate of goal position: ")

        self.start_pos = {"x": initial_x, "y": initial_y}
        self.goal_pos = {"x": goal_x, "y": goal_y}

    def show_pose_sequence(self, locations):
        """Publishes PoseStamped messages and draws relevant markers"""
        marker_id = 0

        for i in range(len(locations) - 1):
            this_location = locations[i]
            next_location = locations[i + 1]

            difference = [next_location["x"] - this_location["x"],
                          next_location["y"] - this_location["y"]]
            angle = math.atan2(difference[1], difference[0])

            # We find the angle to rotate and calculate the modulo so we don't spin in a circle
            angle_to_rotate = angle % (2 * math.pi)

            # we do the faster rotation if the angle is more than 180 degrees
            if angle_to_rotate > math.pi:
                angle_to_rotate = angle_to_rotate - (2 * math.pi)

            # calculate the pose and create the necessary markers and pose stamped messages
            pose = self.get_pose(this_location, angle_to_rotate)
            self.publish_marker(marker_id, pose)
            marker_id += 1
            self.publish_pose_stamped(pose)

        # mark the last position
        pose = self.get_pose(locations[-1], 0)
        self.publish_marker(marker_id, pose)
        self.publish_pose_stamped(pose)

    def plan(self):
        """The planning driver function"""
        print("Beginning planning")

        # gets the initial and final positions
        self.get_positions()

        # transforms the initial/final positions to the grid resolution
        grid_start_pos = {
            "x": int(float(self.start_pos["x"]) / self.grid_msg.info.resolution),
            "y": int(float(self.start_pos["y"]) / self.grid_msg.info.resolution),
        }
        grid_goal_pos = {
            "x": int(float(self.goal_pos["x"]) / self.grid_msg.info.resolution),
            "y": int(float(self.goal_pos["y"]) / self.grid_msg.info.resolution),
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
