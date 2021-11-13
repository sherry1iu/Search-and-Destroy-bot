#!/usr/bin/env python

# Author: Elliot Potter, CS 81, Dartmouth College
# Date: 11/9/2021


import rospy
import tf

FREQUENCY = 10


if __name__ == '__main__':
    """
    Loads the information about the map from a file; does stuff to publish it.
    """
    rospy.init_node('map_broadcaster')
    rate = rospy.Rate(FREQUENCY)
    rospy.sleep(2)

    # TODO -- load map from grid.npy and grid_msg_info.txt

    while not rospy.is_shutdown():
        # TODO -- create a new grid msg, broadcast
        rate.sleep()
