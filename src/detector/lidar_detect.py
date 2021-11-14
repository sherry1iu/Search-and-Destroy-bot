print("LIDAR has been imported.")

import math 
import numpy as np
import tf
from enum import Enum

import rospy # module for ROS APIs

from sensor_msgs.msg import LaserScan                       # LIDAR
from nav_msgs.msg import OccupancyGrid                      # Previously made occupancy grid
from geometry_msgs.msg import PoseWithCovarianceStamped     # AMCL pose
from tf.msg import tfMessage                                # AMCL transformation
from std_msgs.msg import String                             # Mode

#from std_msgsf.msg import Bool    
from std_msgs.msg import Float32     

FREQUENCY = 10

DEFAULT_OCCUGRID_TOPIC = "map"
DEFAULT_SCAN_TOPIC = 'base_scan'

# AMCL Topics
AMCL_POSE_TOPIC = "amcl_pose"

FLOAT32_TOPIC = "angle"
MODE_TOPIC = "mode"


class Lidar_detect:  
    def __init__(self, robx = 0, roby = 0):   # Delete these parameters once we're testing wiht topics.     

        # Laser subscriber from LIDAR
        self.laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self.laser_callback, queue_size=1)

        # Occupancy grid subscriber
        self.occu_sub = rospy.Subscriber(DEFAULT_OCCUGRID_TOPIC, OccupancyGrid, self.occugrid_callback, queue_size=1)

        # Pose from AMCL / localization node
        self.pose_sub = rospy.Subscriber(AMCL_POSE_TOPIC, PoseWithCovarianceStamped, self.pose_callback, queue_size = 1)
        
        # Mode pub/sub
        self.mode_pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)
        self.mode_sub = rospy.Subscriber(MODE_TOPIC, String, self.mode_callback, queue_size=1)

        # Angle publisher
        self.float32_pub = rospy.Publisher(FLOAT32_TOPIC, Float32, queue_size = 1)


        # True until done testing. At which point it's false.
        self.intruder = True #False
        self.intruder_angle = 0


        # These stand in until we implement a good tf and pose subscriber. Then they'll init to none.
        self.resolution = None

        self.robx = robx
        self.roby = roby
        self.yaw = 0


        self.mode_recieved = None
        self.mode_published = "patrolling"
        self.prev_mode_pub = "None"

 
        self.data_ready = False
        print("LIDAR init finished.")

    def mode_callback(self, msg):
        self.mode_recieved = msg.data

    def laser_callback(self, msg):
        
        self.data_ready = False
        self.prev_mode_pub = self.mode_pub
        # Create transformation matrix for map_T_robot by using the robot's pose
        cos = math.cos(self.yaw)
        sin = math.sin(self.yaw)
        
        rob_T_map = np.array([[cos, -1 * sin, 0, self.robx],\
                              [sin, cos, 0, self.roby],\
                              [0, 0, 1, 0],\
                              [0, 0, 0, 1]])
        
        map_T_rob = np.linalg.inv(rob_T_map)

        err_count = 0
        intruder_detected = False

        # For each item in the ranges list
        for i in range(len(msg.ranges)):

            # Current values
            curr_range = msg.ranges[i]
            curr_angle = msg.angle_min + (i * msg.angle_increment)

            # If the laser is in range
            if (curr_range > msg.range_min) and (curr_range < msg.range_max):

                # x, y in robot reference frame
                distx = curr_range * math.cos(curr_angle)
                disty = curr_range * math.sin(curr_angle)


                # Transform robot to map
                map_pts = map_T_rob.dot(np.array([distx, disty, 0, 1]))
                mapx = map_pts[0]
                mapy = map_pts[1]
                

                # Point in grid reference frame (map point in grid units)
                x = int((mapx)/self.resolution) 
                y = int((mapy)/self.resolution) 


                # If the x or y falls outside range, skip it.
                if (x < len(self.grid[0])) and (y < len(self.grid)):
                        

                    # If there is no obstacle in the grid but it has been detected here
                    if self.grid[y][x] == 0:
                        err_count = err_count + 1
                        

                        # An triangle of .01745 rad (msg.angle_increment) and a distance of 150 m has a base of ~2.6 cm
                        # With an ankle of ~22 cm diameter, ankle = ~7 cm diameter, or about 3 increments
                        # Any smaller and the obstacle is considered too far to chase
                        if err_count == 3:
                            intruder_detected = True
                            
                            # If multiple intruders detected, just go after the final one
                            int_angle = curr_angle

                    else:
                        err_count = 0

        if intruder_detected:
            self.intruder = True
            self.intruder_angle = int_angle
            self.mode_published = "chase"
        else:
            if self.prev_mode_pub == "patrolling":
                self.mode_published = "patrolling"
            else:
                self.mode_published = "localize"
            self.intruder = False
            #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            # Later I may decide to add a "data ready", once the actual messages are set up.
            # In that case, then we can just change the intruder_detected straight inside the loop 
            # Then we'd get rid of int_angle and intruder_detected
            # But until then we should keep them to prevent early variables from being detected.
        print("Laser callback finished.")

        

    def occugrid_callback(self, msg):
        
        "Index into this with [y][x]"
        self.grid = np.reshape(msg.data, (msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution
        print("Occugrid callback finished")


    def pose_callback(self, msg):
        # Robot's pose in map reference frame

        pose_map = msg.pose.pose

        self.robx = pose_map.position.x
        self.roby = pose_map.position.y

        quaternion = (pose_map.orientation.x, pose_map.orientation.y, pose_map.orientation.z, pose_map.orientation.w)
        self.yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
        print("Pose callback finished")

    
    def spin(self):
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.

        while not rospy.is_shutdown():

            msg = rospy.wait_for_message(DEFAULT_OCCUGRID_TOPIC, OccupancyGrid)
            #msg = rospy.wait_for_message(DEFAULT_SCAN_TOPIC, LaserScan)


            #if self.mode_recieved == "patrolling"          Reimplement once camera is figured out.

            float32_msg = Float32()
            float32_msg.data = self.intruder_angle
            self.float32_pub.publish(float32_msg)

            mode_msg = String()
            mode_msg.data = self.mode_published
            self.mode_pub.publish(mode_msg)
            
            print("Angle, modes published, recieved")
            print(self.intruder_angle, self.mode_published, self.mode_recieved)

            rate.sleep()

def main():
    """Main function."""

    # 1st. initialization of node.
    rospy.init_node("lidar_detect")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialization of the class
    ld_dt = Lidar_detect()

    # Robot publishes map.
    try:
        ld_dt.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
