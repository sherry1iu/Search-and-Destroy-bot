print("Chaser imported")
import math 

import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel

import sys
sys.path.append('../../')

from src.utilities.move_to_point import *

LINEAR_VELOCITY = .2 # m/s
ANGULAR_VELOCITY = 0.5 # rad/s

FREQUENCY = 10 #Hz.

DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'

# PD controller constants
KP = 1.1     #1
KD = 1     #1
KI = 1.5     #1

INTRUDER_ANG = 0


BOOL_TOPIC = "intruder"
FLOAT32_TOPIC = "angle"

"""
The angle of the camera is about 90 degrees in front of it. So then if we found the midpoint we can do 90/location of midpoint
Use this to control PID
"""
class fsm(Enum):
    ROTATE_CALC = 1
    ROTATE = 2

class Chaser:
    def init(self, angvel = ANGULAR_VELOCITY, linvel = LINEAR_VELOCITY, \
             kp = KP, kd = KD, ki = KI):
    
        # Pass into move
        self.cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)

        # Find angle from lidar
        self.float32_sub = rospy.Subscriber(FLOAT32_TOPIC, Float32, self.angle_callback, queue_size = 1)

        # Subscribe to and publish nodes
        self.mode_sub = rospy.Subscriber(MODE_TOPIC, String, self.mode_callback, queue_size=1)
        self.mode_pub = rospy.Publisher(MODE_TOPIC, String, queue_size=1)

        self.kp = kp
        self.kd = kd
        self.ki = ki

        self.prev_error = 0
        
        self.linear_velocity = linvel
        self.angular_velocity_max = angvel

        self.angular_velocity = 0

        self.intruder_angle = None

        self.FSM = FSM.ROTATE_CALC

        
        self.mode_recieved = None
        self.mode_published = "patrolling"

        print("Chaser init done")

    
    def mode_callback(self, msg):
        self.mode_recieved = msg.data


    def angle_callback(self, msg):
        self.intruder_angle = msg.data


    def spin(self):
        
        while not rospy.is_shutdown():
            if self.mode == "chase"
                #PID P
                p_part = self.intruder_angle

                

                #PID D
                intruder_quadrant = self.intruder_angle // (self.width//5)
                error = (intruder_quadrant / 2) * self.angular_velocity_max

                # Calculate w using approximation of PD controller
                self.angular_velocity = self.kp * error + self.kd * (error - self.prev_error) / (1 / float(FREQUENCY))

                # Prepare for next D
                self.prev_error = error


                move(self.linear_velocity, self.angular_velocity, self.cmd_pub)


def main():
    # # 1st. initialization of node.
    rospy.init_node("node")    
    # # Wait for setup
    rospy.sleep(2)
 
    chaser = Chaser()
    # Camera detects intruder in certain quadrant
    
    try:
        chaser.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    
    


if __name__ == "__main__":
    main()


