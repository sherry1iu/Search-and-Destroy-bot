#!/usr/bin/env python

# Author: Elliot Potter, CS 81, Dartmouth College
# Date: 11/13/2021
import json
import sys

import numpy
sys.path.append('../../')

from src.utilities.get_json_from_grid_msg import get_json_from_grid_msg

import rospy  # module for ROS APIs
from nav_msgs.msg import OccupancyGrid

class MapWriter:
    """Writes an instance of the map to a file
    """

    def __init__(self):
        """Initialization function."""
        self.subscriber = rospy.Subscriber("map", OccupancyGrid, self.grid_callback)
        self.headerLoc = "../test_info/grid_msg_info.txt"
        self.gridLoc = "../test_info/grid.npy"
        self.hasSaved = False
        rospy.sleep(2)

    def grid_callback(self, grid_msg):
        """The callback to set the grid instance variable"""
        if not self.hasSaved:
            self.hasSaved = True
            numpy.save(self.gridLoc, grid_msg.data)
            grid_msg.data = ""
            info = get_json_from_grid_msg(grid_msg)
            with open(self.headerLoc, 'w') as outfile:
                outfile.write(info)


if __name__ == "__main__":
    # 1st. initialization of node.
    rospy.init_node("file_map_writer")

    # 2nd. Creation of the class with relevant publishers/subscribers.
    map_writer = MapWriter()
