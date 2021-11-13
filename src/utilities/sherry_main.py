import rospy 



# Testing for sherry nodes
def main():
    while not rospy.is_shutdown():
        print("Placeholder for global localization. Robot is at (10, 10) on the occugrid.")
        x = 10
        y = 10
        print("Robot is (fake) moving.")

        print("Robot is running LIDAR. Lidar both publishes and returns true/false")
        
        ######### RUN LIDAR DETECT

        obstacle = True ##### Placeholder for now
        
        if obstacle:
            print("Robot found obstacle.")

            cam_count = 0

            while cam_count < 5:

                ########## RUN CAMERA
                ########## RUN PID

                cam_count = cam_count + 1
        
        
        
        print("Restart with localization.")
        print("\n")
                




    


if __name__ == "__main__":
    main()
