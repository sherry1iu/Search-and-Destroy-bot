import rospy 

FREQUENCY = 5 #Hz.

# Testing for sherry nodes
def main():
    rospy.init_node("node")   
    rate = rospy.Rate(FREQUENCY)


    while not rospy.is_shutdown():
        print("Placeholder for global localization. Robot is at (10, 10) on the occugrid.")
        x = 10
        y = 10
        print("Placeholder for moving. Robot is (fake) traversing graph.")

        print("Test for LIDAR. Lidar both publishes and returns true/false")
        
        ######### RUN LIDAR DETECT

        obstacle = True ##### Placeholder for now
        
        if obstacle:
            print("BOOLEAN: Robot found obstacle.")

            cam_count = 0

            print("Test for Camera.")
            print("Test for PID.")
            while cam_count < 5:

                ########## RUN CAMERA
                ########## RUN PID
                

                cam_count = cam_count + 1
                print("Cam count " + str(cam_count))
        
        
        
        print("Restart cycle.")
        print("\n")
        rate.sleep()
                




    


if __name__ == "__main__":
    main()
