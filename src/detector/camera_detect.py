import math 
import numpy as np
import cv2

import sys
print(sys.path)
#import color_tracker

import rospy # module for ROS APIs
from sensor_msgs.msg import CompressedImage

# Check these imports
from scipy.ndimage import filters
import roslib
import sys, time


import sys
sys.path.append('../../')

from src.utilities.move_to_point import *
#from src.utilities.move_to_point import *

#DEFAULT_CAMERA_TOPIC = "/camera/rgb/image_raw/compressed"
#DEFAULT_CAMERA_TOPIC = "/image_publisher_1636692042732699800/image_raw/compressed"
DEFAULT_CAMERA_TOPIC = "/image_publisher_1636697989944373400/image_raw/compressed"

YELLOW_DULLEST = [149, 157, 7]
YELLOW_BRIGHTEST = [237, 250, 1]

GREY_DULLEST = [116,106,96]
GREY_BRIGHTEST = [250, 233, 217]

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

        self.testvar = None
        self.obstacle = False
        self.intruder_mid = None

        print("Camera init done")

    def camera_callback(self, msg):

        # Turns message into a numpy array
        np_arr = np.fromstring(msg.data, np.uint8)
        self.testvar = 1
        print("Started\n")

        # Turn the np image into a cv2 image (image array)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        print(len(image))
        print(len(image[1]))

        # Tuples of (rgb turned into list)
        boundaries = [(YELLOW_DULLEST, YELLOW_BRIGHTEST), (GREY_DULLEST, GREY_BRIGHTEST)]

        for (lower, upper) in boundaries:

            if lower[0] == 149:
                grey = True
            else:
                grey = False


            ## create NumPy arrays from the boundaries
            lower = np.array(lower, dtype = "uint8")
            upper = np.array(upper, dtype = "uint8")
            
            ## find the colors within the specified boundaries and apply mask
            mask = cv2.inRange(image, lower, upper)
            
            if grey:
                print("Iran 77")
                print("grey detected" + str(sum(sum(mask))))
            else:
                print("Iran 80")
                print("yellow detected" + str(sum(sum(mask))))

            
            "IF A LOT OF YELLOW, NEXT TO GREY"################################################################
            if True:
                self.obstacle = True
                rightmost = -1
                leftmost = float('inf')
                for i in sum(mask):
                    if i > rightmost:
                        rightmost = i
                    elif i < leftmost:
                        leftmost = i
                
                ######################################################
                self.intruder_mid = (rightmost + leftmost)/2
            else:
                self.obstacle = False
            


    def spin(self):
        while not rospy.is_shutdown():
            msg = rospy.wait_for_message(DEFAULT_CAMERA_TOPIC, CompressedImage)

            ############################################################################################
            if self.obstacle:
                "ROS SERVICE CALL MIDDLE OF DETECTED OBSTACLE MEDIAN"
                "ROS SERVICE CALL STATE REMAINS AT CHASE"
                pass
            else:
                "ROS SERVICE CALL STATE IS LOCALIZE"
            

                pass



def main():
    # # 1st. initialization of node.
    rospy.init_node("node")    
    # # Wait for setup
    rospy.sleep(2)
 
    follower = Camera_detect()



    try:
        follower.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")
    
    


if __name__ == "__main__":
    main()

