import rospy  # module for ROS APIs
from geometry_msgs.msg import Twist # message type for velocity command.
import math

FREQUENCY = 10
VELOCITY = 0.2 #m/s
ANGULAR_VELOCITY = 0.5 #radians / sec


def move(linear_vel, angular_vel, move_cmd_pub):
    """Send a velocity command (linear vel in m/s, angular vel in rad/s)."""
    # Setting velocities.
    twist_msg = Twist()

    twist_msg.linear.x = linear_vel
    twist_msg.angular.z = angular_vel
    move_cmd_pub.publish(twist_msg)


def run_movement_loop(movement_time, linear_vel, angular_vel, move_cmd_pub):
    """Moves the robot in the specified manner for the specified time"""
    rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
    start_time = rospy.get_rostime()
    while not rospy.is_shutdown():
        if rospy.get_rostime() - start_time >= rospy.Duration(movement_time):
            break

        move(linear_vel, angular_vel, move_cmd_pub)
        rate.sleep()


def rotate(angle_to_rotate, move_cmd_pub):
    """Rotates the robot by a certain angle"""
    time_to_rotate = angle_to_rotate / ANGULAR_VELOCITY
    angular_velocity = ANGULAR_VELOCITY

    # if the angle is negative, the rotation time will be negative.
    # we will flip the sign of rotation time and change angular velocity as well
    reverse_traverse_direction = time_to_rotate < 0
    if reverse_traverse_direction:
        time_to_rotate = -time_to_rotate
        angular_velocity = -angular_velocity
    run_movement_loop(time_to_rotate, 0, angular_velocity)


def move_forward(distance, move_cmd_pub):
    """Moves the robot forward by a certain distance"""
    rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.

    time_to_travel = distance / VELOCITY
    run_movement_loop(time_to_travel, VELOCITY, 0, move_cmd_pub)


def rotate_to_point(current_orientation, current_point, desired_point, move_cmd_pub):
    """
    Rotates in the direction of the specified point
    """
    difference = [desired_point[0] - current_point[0],
                  desired_point[1] - current_point[1]]
    # find the angle that we need to rotate to
    # this is from the reference frame of the previous point, where the +x direction
    # of that point is in the +x direction of the robot when it was at 0,0.
    # the angle is just opp / adj; in this case y / x

    # We use math.atan2 because Python 2 has a stupid issue where
    # it uses integer division for integers.
    # This would be fine in a typed language, but is not when
    # we are dealing with interpreted variables!
    # Fortunately, this is fixed in Python 3.
    angle = math.atan2(difference[1], difference[0])

    # We find the angle to rotate and calculate the modulo so we don't spin in a circle
    angle_to_rotate = (angle - current_orientation) % ( 2 * math.pi )
    # we do the faster rotation if the angle is more than 180 degrees
    if angle_to_rotate > math.pi:
        angle_to_rotate = angle_to_rotate - ( 2 * math.pi )

    # rotate to the angle
    rotate(angle_to_rotate, move_cmd_pub)


def drive_to_point(current_point, desired_point, move_cmd_pub):
    """
    Drives to the point
    """
    # find the difference between the current point and the next one
    difference = [desired_point[0] - current_point[0],
                  desired_point[1] - current_point[1]]

    # find the length that we need to travel
    length = math.sqrt(difference[0] ** 2 + difference[1] ** 2)
    move_forward(length, move_cmd_pub)


def move_to_point(current_point, current_orientation, desired_point, move_cmd_pub):
    """Moves the robot from the current point to the desired point"""
    rotate_to_point(current_orientation, current_point, desired_point, move_cmd_pub)
    drive_to_point(current_point, desired_point, move_cmd_pub)
