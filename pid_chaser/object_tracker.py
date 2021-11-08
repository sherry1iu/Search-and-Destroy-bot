#!/usr/bin/env python

# Author: Elliot Potter, CS 81, Dartmouth College
# Date: 11/8/2021

# import of relevant libraries.
import rospy  # module for ROS APIs
from geometry_msgs.msg import Twist  # message type for velocity command.
from nav_msgs.msg import Odometry
from occupancy_grid_mapping import GridMapper
from sensor_msgs.msg import LaserScan  # message type for scan

import math
import tf  # library for transformations.

FREQUENCY = 10  # Hz.

class ObjectTracker:
    """Tracks an object as the robot moves to intercept"""
    def __init__(self, is_live):
        self.in_pursuit_mode = False

        self.tracked_object_bool_map = None

        self.tracked_object_angle = None
        self.tracked_object_size = None
        self.

    def tracked_object_bool_map_callback(self, bool_map):
        """Takes a boolean map which it uses to find the initial representation of an object"""
        self.tracked_object_bool_map = bool_map

    def camera_callback(self, camera_msg):
        """Takes the output of the RGBD camera; uses it to track the object"""

