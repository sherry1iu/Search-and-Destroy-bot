#!/usr/bin/env python

print("LIDAR has been imported.")

import math
import numpy as np
import tf
from enum import Enum

import rospy # module for ROS APIs

from geometry_msgs.msg import Point                         # For visualization of error/object locations
from sensor_msgs.msg import PointCloud                      # For visualization of error/object locations
from sensor_msgs.msg import LaserScan                       # LIDAR
from nav_msgs.msg import OccupancyGrid                      # Previously made occupancy grid
from geometry_msgs.msg import PoseWithCovarianceStamped     # AMCL pose
from tf.msg import tfMessage                                # AMCL transformation
from std_msgs.msg import String                             # Mode
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud

#from std_msgsf.msg import Bool
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
FREQUENCY = 10

DEFAULT_OCCUGRID_TOPIC = "/blur_map"
DEFAULT_SCAN_TOPIC = 'scan'
DEFAULT_ODOM_TOPIC = "odom"

# AMCL Topics
AMCL_POSE_TOPIC = "amcl_pose"

FLOAT32_TOPIC = "angle"
MODE_TOPIC = "mode"


class Lidar_detect:
    def __init__(self, is_live):   # Delete these parameters once we're testing wiht topics.

        self.test_callbacks = ["mode", "laser", "occugrid", "pose", "odom"]


        # Occupancy grid subscriber
        self.occu_sub = rospy.Subscriber(DEFAULT_OCCUGRID_TOPIC, OccupancyGrid, self.occugrid_callback, queue_size=1)

        # Pose from AMCL / localization node
        self.pose_sub = rospy.Subscriber(AMCL_POSE_TOPIC, PoseWithCovarianceStamped, self.pose_callback, queue_size = 1)

        # Before localization node is determined, use this one
        self._odom = rospy.Subscriber(DEFAULT_ODOM_TOPIC, Odometry, self.odom_callback, queue_size=1)

        # Mode pub/sub
        self.mode_pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)
        self.mode_sub = rospy.Subscriber(MODE_TOPIC, String, self.mode_callback, queue_size=1)

        # Angle publisher
        self.float32_pub = rospy.Publisher(FLOAT32_TOPIC, Float32, queue_size = 1)


        # True until done testing. At which point it's false.
        self.intruder = True #False
        self.intruder_angle = 0

        self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size = 100)
        rospy.wait_for_message(DEFAULT_OCCUGRID_TOPIC, OccupancyGrid)
        rospy.wait_for_message(DEFAULT_ODOM_TOPIC, Odometry)

        self.error_pub = rospy.Publisher('errors', PointCloud, queue_size=10)

        if is_live:
            self.subscriber = rospy.Subscriber("scan", LaserScan, self.laser_callback)
            self.laser_frame = "laser"
        else:
            self.subscriber = rospy.Subscriber("base_scan", LaserScan, self.laser_callback)
            self.laser_frame = "base_laser_link"


        # Initialization
        self.resolution = None
        self.origin = None

        self.mode_recieved = None
        self.mode_published = "patrolling"
        self.prev_mode_pub = "None"

        self.msg = None

        self.data_ready = False

        # move this down so that laser doesn't start early
        # Laser subscriber from LIDAR
        self.laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self.laser_callback, queue_size=1)

        print("LIDAR init finished.")
        self.t = tf.TransformListener(True, cache_time=rospy.Duration(10))

    def tf_callback(self, msg):
        # Odom to map

        odom_T_map = msg.transforms.transform

        self.odom_trans = (odom_T_map.translation.x, odom_T_map.translation.y) # z is always 0

        quaternion = (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
        self.odom_rot = tf.transformations.euler_from_quaternion(quaternion)[2]

    def publish_errors(self, point_array):
        """Publishes all the error points detected by the LIDAR"""
        cloud = PointCloud()
        cloud.header.frame_id = "map"
        cloud.points = point_array
        self.error_pub.publish(cloud)


    def mode_callback(self, msg):
        self.mode_recieved = msg.data
        self.test_callbacks[0] = "1"

    def laser_callback(self, msg):
        self.msg = msg

    def laser_fx(self, msg):
        (trans, rot) = self.t.lookupTransform('odom', self.laser_frame, rospy.Time(0))
        t = tf.transformations.translation_matrix(trans)
        R = tf.transformations.quaternion_matrix(rot)

        odom_T_laserFrame = t.dot(R)
        # baseLink_T_odom = np.linalg.inv(odom_T_laserFrame)

        err_count = 0
        intruder_detected = False

        count = 0
        max_count = 0
        not_printed = True
        error_array = []


        for i in range(len(msg.ranges)):
            scan_range = msg.ranges[i]
            scan_angle = msg.angle_min + i * msg.angle_increment

            if (scan_range > msg.range_min) and (scan_range < msg.range_max):
                # get the position in the ref frame of the robot
                target_location = {"x": scan_range * math.cos(scan_angle), "y": scan_range * math.sin(scan_angle)}
                # get the position in the odom ref frame
                new_vertex_np = odom_T_laserFrame.dot(
                    np.array([target_location["x"], target_location["y"], 0, 1])
                )

                #count = count + 1

                x = int((new_vertex_np[0] - self.origin.position.x) / self.resolution)
                y = int((new_vertex_np[1] - self.origin.position.y) / self.resolution)

                # scan_range = msg.ranges[i] + .2

                #print(x, y, self.grid[x][y])
                if self.grid[y][x] == 0:
                    if not_printed == True:
                        print("Start here")
                        print(x, y)
                        print(i)
                        print(scan_range)
                        not_printed = False
                    count += 1
                    #print(i)
                    #print(scan_range)
                    x_max = x
                    y_max = y
                    max_count += 1


                # If the x or y falls outside range, skip it.
                if (x < len(self.grid[0])) and (y < len(self.grid)):

                    #print("(x, y) is " + str((x, y)))
                    ################################################################################################################################ See if we can rerun the opccupancy grid generator. It's finding obstacles somewhere it shouldn't
                    # If there is no obstacle in the grid but it has been detected here
                    if self.grid[y][x] != 100:
                        err_count = err_count + 1

                        error_point = Point()
                        error_point.x = new_vertex_np[0] 
                        error_point.y = new_vertex_np[1]
                        error_point.z = 0.5
                        error_array.append(error_point)

                        #print(mapx, mapy)


                        # An triangle of .01745 rad (msg.angle_increment) and a distance of 150 m has a base of ~2.6 cm
                        # With an ankle of ~22 cm diameter, ankle = ~7 cm diameter, or about 3 increments
                        # Any smaller and the obstacle is considered too far to chase
                        #if err_count == 3:
                        intruder_detected = True
                        #while True:
                        #    print()
                        #    print(x, y, self.grid[y][x])
                        #    print(mapx, mapy)


                        # If multiple intruders detected, just go after the final one
                        int_angle = scan_angle

                    #else:
                        #err_count = 0
                #else:
                    #print("(" + str(x) + ", " + str(y) + ") skipped")
                #if (scan_angle > 45*math.pi/180):
                #    print(self.robx, self.roby, self.yaw)
                #    while True:

                        #pass\

        self.publish_errors(error_array)

        print("Error count:")
        print(err_count)
        print(count)
        print(max_count)
        #print(len(msg.ranges))
        if intruder_detected:
            self.intruder = True
            # transform the angle back to the correct reference frame
            self.intruder_angle = tf.transformations.euler_from_quaternion(rot)[2] + int_angle
            self.mode_published = "chaser"
        else:
            if self.prev_mode_pub == "chaser":
                self.mode_published = "localizing"
            else:

                self.mode_published = "patrolling"
            self.intruder = False
            #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            # Later I may decide to add a "data ready", once the actual messages are set up.
            # In that case, then we can just change the intruder_detected straight inside the loop
            # Then we'd get rid of int_angle and intruder_detected
            # But until then we should keep them to prevent early variables from being detected.



        #print(count)
        print("finished")
        print(self.mode_published)
        print(self.intruder)
        print(intruder_detected)
        #while True:
        #    pass
        #    self.marker_pub.publish(marker_msg)


        self.test_callbacks[1] = "1"



    def occugrid_callback(self, msg):
        "Index into this with [y][x]"
        self.grid = np.reshape(msg.data, (msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        self.origin = msg.info.origin
        self.test_callbacks[2] = "1"


    def pose_callback(self, msg):
        # Robot's pose in map reference frame

        pose_map = msg.pose.pose

        self.robx = pose_map.position.x
        self.roby = pose_map.position.y

        quaternion = (pose_map.orientation.x, pose_map.orientation.y, pose_map.orientation.z, pose_map.orientation.w)
        self.yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
        self.test_callbacks[3] = "1"


    def odom_callback(self, msg):
        # Callback function to get pose from odom

        # X and Y positions
        self.robx = msg.pose.pose.position.x
        self.roby = msg.pose.pose.position.y

        # Yaw angle
        quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        self.yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
        self.test_callbacks[4] = "1"



    def spin(self):
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
        count = 0

        while not rospy.is_shutdown():

            msg = rospy.wait_for_message(DEFAULT_OCCUGRID_TOPIC, OccupancyGrid)
            #msg = rospy.wait_for_message(DEFAULT_SCAN_TOPIC, LaserScan)
            if self.msg != None:
                self.laser_fx(self.msg)

                #if self.mode_recieved == "patrolling"          Reimplement once camera is figured out.

                float32_msg = Float32()
                float32_msg.data = self.intruder_angle
                self.float32_pub.publish(float32_msg)

                mode_msg = String()
                mode_msg.data = self.mode_published
                self.mode_pub.publish(mode_msg)

                #print("Angle, modes published, recieved" + str((self.intruder_angle, self.mode_published, self.mode_recieved)) + str(self.test_callbacks))

                rate.sleep()

def main():
    """Main function."""

    # 1st. initialization of node.
    rospy.init_node("lidar_detect")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialization of the class
    ld_dt = Lidar_detect(is_live=True)

    # Robot publishes map.
    try:
        ld_dt.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
