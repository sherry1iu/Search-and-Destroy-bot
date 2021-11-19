#!/usr/bin/env python

import math 
from enum import Enum

import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from std_msgs.msg import Float32
from std_msgs.msg import String                             # Mode


import sys
sys.path.append('../../')

from src.utilities.move_to_point import move

LINEAR_VELOCITY = .2 # m/s
ANGULAR_VELOCITY = 0.5 # rad/s

FREQUENCY = 10 #Hz.

DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'

# PD controller constants
KP = 1.1
KI = -.5
KD = .7


#BOOL_TOPIC = "intruder"
MODE_TOPIC = "mode"

FLOAT32_TOPIC = "angle"

"""
The angle of the camera is about 90 degrees in front of it. So then if we found the midpoint we can do 90/location of midpoint
Use this to control PID
"""
class FSM(Enum):
    ROTATE_CALC = 1
    ROTATE = 2
print("Chaser imported")

class Chaser:
    def __init__(self, angvel = ANGULAR_VELOCITY, linvel = LINEAR_VELOCITY, \
             kp = KP, kd = KD, ki = KI):
        # Velocity publisher; passed into move function
        self.cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        # Find angle from lidar/camera
        self.float32_sub = rospy.Subscriber(FLOAT32_TOPIC, Float32, self.angle_callback, queue_size = 1)
        # Find mode (overarching fsm)
        self.mode_sub = rospy.Subscriber(MODE_TOPIC, String, self.mode_callback, queue_size=1)
        self.mode = "patrolling"

        # PID Parameters

        # P part
        self.kp = kp
        self.intruder_angle = 0

        # I part
        self.ki = ki
        self.err_sum = 0

        # D part
        self.kd = kd
        self.prev_error = 0
        self.prev_angle = 0
        
                
        # Amount of movement
        self.FSM = FSM.ROTATE_CALC
        self.linear_velocity = linvel
        self.angular_velocity_max = angvel
        self.angular_velocity = 0
        

        print("Chaser init done")

    
    def mode_callback(self, msg):
        self.mode = msg.data
        print("New mode: " + self.mode)


    def angle_callback(self, msg):
        self.prev_angle = self.intruder_angle
        self.intruder_angle = msg.data


    def spin(self):
        print("91 Ran")
        while not rospy.is_shutdown():
            if self.mode == "chaser":
                msg = rospy.wait_for_message(FLOAT32_TOPIC, Float32)

                # PID P
                p_part = self.kp * (self.intruder_angle) * -1
                
                # PID I
                i_part = self.ki * (self.err_sum)

                # PID D
                d_part = self.kd * (self.intruder_angle - self.prev_angle)

                print("PID: " + str((p_part, i_part, d_part)))


                # Calculate w using approximation of PID controller
                self.angular_velocity = p_part + i_part + d_part

                # Move
                move(self.linear_velocity, self.angular_velocity, self.cmd_pub)##########################################################################################

                # Prepare for next iteration
                self.prev_angle = self.intruder_angle
                self.err_sum = self.err_sum + (self.intruder_angle - self.prev_angle)
            
            else:
                # Reset
                self.prev_angle = 0
                self.err_sum = 0






def main():
    # # 1st. initialization of node.
    print("122 Ran")

    rospy.init_node("node")    
    # # Wait for setup
    print("126 Ran")
    rospy.sleep(2)
    print("Ran 125")
    chaser = Chaser()
    
    try:
        chaser.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    
    


if __name__ == "__main__":
    main()


