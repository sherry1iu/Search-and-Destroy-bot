#!/usr/bin/env python
#The line above is important so that this file is interpreted with Python when running it.

# Author: Elliot Potter, CS 81, Dartmouth College
# Date: 9/17/2021

# Import of python modules.
import math # use of pi.
import random
import tf

# import of relevant libraries.
import rospy # module for ROS APIs
from geometry_msgs.msg import Twist # message type for cmd_vel
from std_msgs.msg import Bool # message type for rotation_warning
from sensor_msgs.msg import LaserScan # message type for scan

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'scan'
DEFAULT_LASER_FRAME = 'laser'

# Frequency at which the loop operates
FREQUENCY = 10 #Hz.

# Velocities that will be used (feel free to tune)
LINEAR_VELOCITY = 0.2 # m/s
ANGULAR_VELOCITY = math.pi/8 # rad/s

# Threshold of minimum clearance distance (feel free to tune)
MIN_THRESHOLD_DISTANCE = 0.5 # m, threshold distance, should be smaller than range_max

# Field of view in radians that is checked in front of the robot (feel free to tune)
MIN_SCAN_ANGLE_DEG = -30.0
MAX_SCAN_ANGLE_DEG = +30.0
MIN_ROTATION_ANGLE = -math.pi
MAX_ROTATION_ANGLE = +math.pi


class RandomWalk():
    def __init__(self, linear_velocity=LINEAR_VELOCITY, angular_velocity=ANGULAR_VELOCITY, min_threshold_distance=MIN_THRESHOLD_DISTANCE,
                 scan_angle_deg=[MIN_SCAN_ANGLE_DEG, MAX_SCAN_ANGLE_DEG], rotation_angle=[MIN_ROTATION_ANGLE, MAX_ROTATION_ANGLE]):
        """Constructor."""

        # Setting up publishers/subscribers.
        # Setting up the publisher to send velocity commands.
        self._cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)

        # publishes a warning message when it's turning
        self.rot_pub = rospy.Publisher("rotation_warning", Bool, queue_size=1)

        # Setting up subscriber receiving messages from the laser.
        self._laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self._laser_callback, queue_size=1)

        # Parameters.
        self.linear_velocity = linear_velocity # Constant linear velocity set.
        self.angular_velocity = angular_velocity # Constant angular velocity set.
        self.min_threshold_distance = min_threshold_distance
        self.rotation_angle = rotation_angle
        self.scan_angle_deg = scan_angle_deg

        # Flag used to control the behavior of the robot.
        self._close_obstacle = False # Flag variable that is true if there is a close obstacle.
        self.t = tf.TransformListener(True, cache_time=rospy.Duration(10))

    def move(self, linear_vel, angular_vel):
        """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
        # Setting velocities.
        twist_msg = Twist()

        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel
        self._cmd_pub.publish(twist_msg)

    def stop(self):
        """Stop the robot."""
        twist_msg = Twist()
        self._cmd_pub.publish(twist_msg)

    def _laser_callback(self, msg):
        """Processing of laser message."""
        # Access to the index of the measurement in front of the robot.
        # NOTE: assumption: the one at angle 0 corresponds to the front.

        _close_obstacle = False

        # Find the minimum range value between min_scan_angle and
        # max_scan_angle
        # If the minimum range value is closer to min_threshold_distance, change the flag self._close_obstacle
        # Note: You have to find the min index and max index.
        # Please double check the LaserScan message http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html
        ######: ANSWER CODE BEGIN #######
        # The range in front of the robot is +/- 90 deg, with -90deg at index 0. This section has 720 measurements
        # 4 per degree.
        # print "Min: " + str(msg.angle_min) + ", Max: " + str(msg.angle_max)
        # print len(msg.ranges)
        (trans, rot) = self.t.lookupTransform('base_link', DEFAULT_LASER_FRAME, rospy.Time(0))
        scan_yaw = tf.transformations.euler_from_quaternion(rot)[2]

        # rectify the scan angles to be in the laser reference frame
        rect_scan_angle_rad = [self.scan_angle_deg[0] * math.pi / 180 + scan_yaw, self.scan_angle_deg[1] * math.pi / 180 + scan_yaw]

        # prevent the scan angles from exceeding the maximum or minimum angle bounds
        for i in range(len(rect_scan_angle_rad)):
            if rect_scan_angle_rad[i] < msg.angle_min:
                rect_scan_angle_rad[i] = 2*math.pi + rect_scan_angle_rad[i]
            if rect_scan_angle_rad[i] > msg.angle_max:
                rect_scan_angle_rad[i] = rect_scan_angle_rad[i] - 2*math.pi

        min_scan_index = int((rect_scan_angle_rad[0] - msg.angle_min) / msg.angle_increment)
        max_scan_index = int((rect_scan_angle_rad[1] - msg.angle_min) / msg.angle_increment)

        if max_scan_index > min_scan_index:
            for laser_range in msg.ranges[int(min_scan_index) : int(max_scan_index)]:
                if (laser_range < self.min_threshold_distance):
                    _close_obstacle = True
                    break
        else:
            for laser_range in (msg.ranges[:int(max_scan_index)] + msg.ranges[int(min_scan_index):]):
                if (laser_range < self.min_threshold_distance):
                    _close_obstacle = True
                    break

        self._close_obstacle = _close_obstacle

            ####### ANSWER CODE END #######

    def publish_rotation_warning(self, is_rotating):
        """Publishes a warning if the robot is rotating"""
        warning = Bool()
        warning.data = is_rotating
        self.rot_pub.publish(warning)

    def spin(self):
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
        while not rospy.is_shutdown():
            # Keep looping until user presses Ctrl+C

            # If the flag self._close_obstacle is False, the robot should move forward.
            # Otherwise, the robot should rotate for a random amount of time
            # after which the flag is set again to False.
            # Use the function move already implemented, passing the default velocities saved in the corresponding class members.
            if (self._close_obstacle):
                angle_to_rotate = random.uniform(self.rotation_angle[0], self.rotation_angle[1])
                # we will rotate a little less than this b/c acceleration2*
                time_to_rotate = angle_to_rotate / self.angular_velocity

                start_time = rospy.get_rostime()
                reverse_traverse_direction = False

                # we can calculate a negative time to rotate if we calculate from a minimum angle.
                if angle_to_rotate < 0:
                    reverse_traverse_direction = True
                    time_to_rotate = -time_to_rotate

                while not rospy.is_shutdown():
                    if rospy.get_rostime() - start_time >= rospy.Duration(time_to_rotate):
                        break

                    if not reverse_traverse_direction:
                        self.publish_rotation_warning(True)
                        self.move(0, ANGULAR_VELOCITY)
                    else:
                        self.publish_rotation_warning(True)
                        self.move(0, -ANGULAR_VELOCITY)

                    rate.sleep()

                    self._close_obstacle = False
            else:
                self.publish_rotation_warning(False)
                # We're only going to move forward for a single tick...
                self.move(LINEAR_VELOCITY, 0)

            rate.sleep()

def main():
    """Main function."""

    # 1st. initialization of node.
    rospy.init_node("random_walk")

    # Sleep for a few seconds to wait for the registration.
    rospy.sleep(2)

    # Initialization of the class for the random walk.
    random_walk = RandomWalk()

    rospy.sleep(1)
    print ("Ready to go!")

    # If interrupted, send a stop command before interrupting.
    rospy.on_shutdown(random_walk.stop)

    # Robot random walks.
    try:
        random_walk.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")


if __name__ == "__main__":
    """Run the main function."""
    main()
