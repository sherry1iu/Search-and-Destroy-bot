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

class PIDChaser:
    """Steers the robot toward the target object"""
    def __init__(self, is_live):

    def main(self):

if __name__ == "__main__":
    # 1st. initialization of node.
    rospy.init_node("pid_chaser")

    # 2nd. Creation of the class with relevant publishers/subscribers.
    motions = PIDChaser(is_live=True)

    # 3rd loop.
    # mapper.main()
    motions.main()