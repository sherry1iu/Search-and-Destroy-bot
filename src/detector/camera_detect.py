#!/usr/bin/env python

import math 
import numpy as np
import cv2

import sys
#print(sys.path)

import rospy # module for ROS APIs
from sensor_msgs.msg import CompressedImage

# Check these imports
from scipy.ndimage import filters
import roslib
import time

from std_msgs.msg import Float32
from std_msgs.msg import String                             # Mode

sys.path.append('../../')

from src.utilities.move_to_point import *
#from src.utilities.move_to_point import *

DEFAULT_CAMERA_TOPIC = "/camera/rgb/image_raw/compressed"
#DEFAULT_CAMERA_TOPIC = "/image_publisher_1636692042732699800/image_raw/compressed"
#DEFAULT_CAMERA_TOPIC = "/image_publisher_1636973670731858300/image_raw/compressed"


FLOAT32_TOPIC = "angle"
MODE_TOPIC = "mode"

YELLOW_DULLEST = [149, 157, 31]
YELLOW_BRIGHTEST = [255,253,140]


GREY_DULLEST = [116,106,96]
GREY_BRIGHTEST = [250, 233, 217]

ORANGE_BRIGHTEST = [153, 44, 19]
ORANGE_MED = [89, 36, 19]
ORANGE_DULLEST = [59, 26, 11]



# Size of rosbot camera and of mask
HEIGHT = 480
WIDTH = 640

'''
Before starting! Open these links.
https://docs.ros.org/en/hydro/api/cv_bridge/html/python/index.html
https://www.pyimagesearch.com/2014/08/04/opencv-python-color-detection/
https://towardsdatascience.com/building-a-color-recognizer-in-python-4783dfc72456
https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/
https://stackoverflow.com/questions/22588146/tracking-white-color-using-python-opencv
https://www.circuitmix.com/detecting-and-tracking-colours-using-python/
https://www.geeksforgeeks.org/python-opencv-imdecode-function/

'''


class Camera_detect:  
    def __init__(self):        
        
        self.camera_sub = rospy.Subscriber(DEFAULT_CAMERA_TOPIC, CompressedImage, self.camera_callback, queue_size=1)

        self.image_pub = rospy.Publisher("/output/image_raw/compressed", CompressedImage, queue_size=1)

        # Mode pub/sub
        self.mode_sub = rospy.Subscriber(MODE_TOPIC, String, self.mode_callback, queue_size=1)

        # Angle publisher
        self.float32_pub = rospy.Publisher(FLOAT32_TOPIC, Float32, queue_size = 1)

        self.intruder_mid = WIDTH/2
        
        self.mode_recieved = "initializing"


        self.obstacle = False
        self.intruder_angle = 0

        # Initialization; changed in callback
        self.height = HEIGHT
        self.width = WIDTH


        self.data_ready = False

    def mode_callback(self, msg):
        self.mode_recieved = msg.data
        print("New mode: " + self.mode_recieved)

    def camera_callback(self, msg):
        """
        Read image like this
        ---------> + x
        |
        |
        v
        +y
        """
        # We completely skip processing if we're making our way back to the graph
        if self.mode_recieved == "restoring":
            return

        self.data_ready = False
            # Turns message into a numpy array
        np_arr = np.fromstring(msg.data, np.uint8)
        self.testvar = 1
        #print("Started Camera callback")

        # Turn the np image into a cv2 image (image array)
        BGR_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # BGR to RGB
        image = cv2.cvtColor(BGR_image, cv2.COLOR_BGR2RGB)

        self.height = len(image)
        self.width = len(image[1])

        # Tuples of rgb ranges; depends on what to test for
        boundaries = [(ORANGE_DULLEST, ORANGE_BRIGHTEST)]
        #boundaries = [(YELLOW_DULLEST, YELLOW_BRIGHTEST)]

        # Range of colors
        lower = boundaries[0][0]
        upper = boundaries[0][1]


        ## create NumPy arrays from the boundaries
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")
        
        ## find the colors within the specified boundaries and apply mask
        mask = cv2.inRange(image, lower, upper)

        # If enough orange found
        if sum(sum(mask)) > 20:

            self.obstacle = True

            rightmost = -1
            leftmost = float('inf')

            zerocount = 0
            two55c = 0
            othercount = 0
            for i in range(len(sum(mask))):
                # Sum of mask is 
                #print(sum(mask[i]))
                if sum(mask)[i] > 20:
                    if i > rightmost:
                        rightmost = i
                    if i < leftmost:
                        leftmost = i
                if sum(mask)[i] == 0:
                    zerocount += 1
                elif sum(mask)[i] == 255:
                    two55c += 1
                    
                else:
                    othercount += 1
                    #print(sum(mask)[i])
            ######################################################
            self.intruder_mid = (rightmost + leftmost)/2
            print(self.intruder_mid)
            '''
            print(182)
            print(leftmost, rightmost)
            print(zerocount, two55c, othercount)
            '''
        else:
            self.obstacle = False

        #self.obstacle = False########################################
        self.data_ready = True
        #print("End of callback")
            


    def spin(self):

        total = math.pi/2
        msg = rospy.wait_for_message(DEFAULT_CAMERA_TOPIC, CompressedImage)
        while not rospy.is_shutdown():

            if self.obstacle:

                fraction_location = float(self.intruder_mid) / self.width
                
                self.intruder_angle = (total * fraction_location) - (total)/2

            if self.intruder_angle is not None:
                print("Intruder angle: " + str(self.intruder_angle))
                float32_msg = Float32()
                float32_msg.data = self.intruder_angle
                self.float32_pub.publish(float32_msg)
           
            else:
                self.intruder_angle = None

            rospy.sleep(1.0)



def main():
    # # 1st. initialization of node.
    rospy.init_node("n0de")    
    # # Wait for setup
    rospy.sleep(2)
 
    camdet = Camera_detect()

    try:
        camdet.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    main()

