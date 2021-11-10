#!/usr/bin/env python

# Author: Elliot Potter, CS 81, Dartmouth College
# Date: 11/9/2021


import rospy
import tf

FREQUENCY = 10


if __name__ == '__main__':
    """
    Publishes transformations to the map reference frame

    x_offset: the offset the MAP has with respect to ODOM
    y_offset: the offset the MAP has with respect to ODOM
    yaw_offset: the offset the MAP has with respect to ODOM
    """
    rospy.init_node('fixed_tf_broadcaster')

    rate = rospy.Rate(FREQUENCY)
    br = tf.TransformBroadcaster()

    while not rospy.is_shutdown():
        br.sendTransform((0, 0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 0),
                         rospy.Time.now(),
                         "map",
                         "odom"
                         )
        rate.sleep()
