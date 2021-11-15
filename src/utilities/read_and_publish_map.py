#!/usr/bin/env python

# Author: Isaac Feldman, CS 81, Dartmouth College
# Date: 11/9/2021


import rospy
import numpy as np
import json
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

from scipy import ndimage

FREQUENCY = 10
PUBLISH_TOPIC = "/static_map"
BLUR_TOPIC = "/blur_map"



if __name__ == '__main__':
    """
    Loads the information about the map from a file; does stuff to publish it.
    """
    rospy.init_node('map_broadcaster')
    rate = rospy.Rate(FREQUENCY)
    rospy.sleep(2)

    data = np.load("./src/test_info/grid.npy")                                 # Load serialized map from file
    with open("./src/test_info/grid_msg_info.txt", 'r') as fp:
        info = json.loads(fp.read())
    blurred_data = 100 * ndimage.binary_dilation(data)

    static_map_pub = rospy.Publisher(PUBLISH_TOPIC, OccupancyGrid, queue_size=1)
    blurred_map_pub = rospy.Publisher(BLUR_TOPIC, OccupancyGrid, queue_size=1)


while not rospy.is_shutdown():
        # Static map
        msg = OccupancyGrid()
        msg.info.origin = Pose()
        msg.data = data
        msg.info.resolution = float(info["info"]["resolution"])
        msg.info.width = int(info["info"]["width"])
        msg.info.height = int(info["info"]["height"])
        x, y, z = info["info"]["origin"]["position"]["x"], info["info"]["origin"]["position"]["y"], info["info"]["origin"]["position"]["z"]
        msg.info.origin.position.x = x
        msg.info.origin.position.y = y
        msg.info.origin.position.z = z
        x, y, z, w = info["info"]["origin"]["orientation"]["x"], info["info"]["origin"]["orientation"]["y"], info["info"]["origin"]["orientation"]["z"], info["info"]["origin"]["orientation"]["w"]
        msg.info.origin.orientation.x = x
        msg.info.origin.orientation.y = y
        msg.info.origin.orientation.z = z
        msg.info.origin.orientation.w = w

        static_map_pub.publish(msg)


        # Blur the same map with a gaussian filter the map and publish
        msg.data = blurred_data
        blurred_map_pub.publish(msg)








        rate.sleep()


