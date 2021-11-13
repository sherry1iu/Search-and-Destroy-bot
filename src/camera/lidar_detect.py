print("LIDAR has been imported.")
import math 
import numpy as np
import tf

import rospy # module for ROS APIs

from sensor_msgs.msg import LaserScan                       # LIDAR
from nav_msgs.msg import OccupancyGrid                      # Previously made occupancy grid
from geometry_msgs.msg import PoseWithCovarianceStamped     # AMCL pose
from tf.msg import tfMessage                                # AMCL transformation


#from nav_msgs.msg import Odometry



DEFAULT_OCCUGRID_TOPIC = "map"
#DEFAULT_ODOM_TOPIC = "odom"

##DEFAULT_SCAN_TOPIC = 'scan'
##LASER_TOPIC = 'laser'
DEFAULT_SCAN_TOPIC = 'base_scan'

# Ngl not sure what this is used for ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
LASER_TOPIC = 'base_laser_link'

# AMCL Topics
AMCL_POSE_TOPIC = "amcl_pose"
#TF_TOPIC = "tf"         #I might end up not actually using this


class Lidar_detect:  
    def __init__(self, robx = 0, roby = 0):        

        # Laser subscriber from LIDAR
        self.laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self.laser_callback, queue_size=1)

        # Occupancy grid subscriber
        self.occu_sub = rospy.Subscriber(DEFAULT_OCCUGRID_TOPIC, OccupancyGrid, self.occugrid_callback, queue_size=1)

        # AMCL subscribers
        self.pose_sub = rospy.Subscriber(AMCL_POSE_TOPIC, PoseWithCovarianceStamped, self.pose_callback, queue_size = 1)
        #self.tf_sub = rospy.Subscriber(TF_TOPIC, tfMessage, self.tf_callback, queue_size = 1)


        # Instance variables
        
        # True until done testing. At which point it's false.
        self.intruder = True #False
        self.intruder_angle = 0


        # These stand in until we implement a good tf and pose subscriber. Then they'll init to none.
        self.resolution = None

        self.robx = robx
        self.roby = roby
        self.yaw = 0
        #self.odom_trans = 0
        #self.odom_rot = 0

 
        print("LIDAR init finished.")


    def laser_callback(self, msg):
        
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
        else:
            self.intruder = False#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
            # Later I may decide to add a "data ready", once the actual messages are set up.
            # In that case, then we can just change the intruder_detected straight inside the loop 
            # Then we'd get rid of int_angle and intruder_detected
            # But until then we should keep them to prevent early variables from being detected.
        

    def occugrid_callback(self, msg):
        
        "Index into this with [y][x]"
        self.grid = np.reshape(msg.data, (msg.info.height, msg.info.width))
        self.resolution = msg.info.resolution


    def pose_callback(self, msg):
        # Robot's pose in map reference frame

        pose_map = msg.pose.pose

        self.robx = pose_map.position.x
        self.roby = pose_map.position.y

        quaternion = (pose_map.orientation.x, pose_map.orientation.y, pose_map.orientation.z, pose_map.orientation.w)
        self.yaw = tf.transformations.euler_from_quaternion(quaternion)[2]
        

        
    def spin(self):
        return(self.intruder, self.intruder_angle)



    '''

    I might not actually need odom to map transformations. 
    Keep this in case for now, but comment it out.

    def tf_callback(self, msg):
        # Odom to map

        odom_T_map = msg.transforms.transform
        
        self.odom_trans = (odom_T_map.translation.x, odom_T_map.translation.y) # z is always 0

        quaternion = (transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
        self.odom_rot = tf.transformations.euler_from_quaternion(quaternion)[2]
    '''

"""
To test this:

publish an occupancy grid (PA3)
get the pose straight from the initial stuff from initial position in PA3. Or see if their pose_stamped works


"