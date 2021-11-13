print("\n")
import rospy 

# Import other nodes
import sys
sys.path.append('../../')

from src.camera.lidar_detect import Lidar_detect
from src.camera.camera_detect import Camera_detect
from src.chaser.chaser import Chaser


FREQUENCY = 5 #Hz.

# Testing for sherry nodes
def main():

    # Init rospy nodes
    rospy.init_node("node")   
    rate = rospy.Rate(FREQUENCY)

    print("\nStart:\n")

    #while not rospy.is_shutdown():
    print("Placeholder for global localization. Robot is at (10, 10) on the occugrid.")
    x = 10
    y = 10

    print("Placeholder for moving. Robot is (fake) traversing graph.")

    print("\nTest for LIDAR. Lidar both publishes and returns true/false")
    lidet = Lidar_detect(x, y)
    (obstacle, angle) = lidet.spin()
    
    ######### RUN LIDAR DETECT

    obstacle = True ##### Placeholder for now
    
    if obstacle:
        print("BOOLEAN: Robot found obstacle.")

        cam_count = 0

        print("\nTest for Camera.")
        print("Test for PID.\n")

        chaser = Chaser()
        
        while obstacle:

            ########## RUN CAMERA
            #camera.spin()

            (obstacle, angle) = lidet.spin()

            ########## RUN PID
            chaser.spin(angle)


    else:
        print("Not obstacle.")
    
    
    
    print("\nRestart cycle.")
    print("\n")
    rate.sleep()

    print("\n\nNotes!\n")
    print("When doing the final test of LIDAR, start the PA3 occupancy grid running.")
    print("Then find the pose of the robot on that occupancy grid and use it for x and y to test for correct.")
    print("Then use some random value of x and y to test for yes intruder")
                


if __name__ == "__main__":
    main()
