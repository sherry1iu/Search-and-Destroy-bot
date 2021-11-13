#!/usr/bin/env python

# Localizer module
# Uses laser scanner and occupancy grid to localize robot

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf
import math
import numpy as np
from time import sleep
import random
from enum import Enum

# Constants.
# Topic names
DEFAULT_CMD_VEL_TOPIC = 'cmd_vel'
DEFAULT_SCAN_TOPIC = 'base_scan'
ODOM_TOPIC = 'odom'
GRID_TOPIC = 'map'

FREQUENCY = 10 #Hz.

# Field of view in radians that is checked in front of the robot
MIN_SCAN_ANGLE_RAD = -math.pi
MAX_SCAN_ANGLE_RAD = math.pi

class fsm(Enum):
    UNINIT = 0
    SCAN = 1
    MOVE = 2
    STOP = 3

class Grid:
    def __init__(self, occupancy_grid_data, width, height, resolution):
        self.grid = np.reshape(occupancy_grid_data, (height, width))
        self.width = width
        self.height = height
        self.resolution = resolution

    def cell_at(self, x, y):
        return self.grid[y, x]

class Position:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class Localizer():
    def __init__(self):
        """Constructor"""
        self.state = fsm.UNINIT

        self.grid = None
        self.distances = None
        self.possible_positions = None
        self.error_threshold = None

        self.last_translation = 0
        self.last_rotation = 0

        self.grid_sub = rospy.Subscriber(GRID_TOPIC, OccupancyGrid, self.grid_callback, queue_size=1)
        self.laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self.laser_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)

    def grid_callback(self, msg):
        if self.state != fsm.UNINIT: return
        self.grid = Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution)
        self.error_threshold = (2 * self.grid.resolution) ** 2
        self.build_distances()
        self.state = fsm.SCAN

    def build_distances(self):
        """Builds a dictionary of forward/back/left/right distances from every grid square"""
        if self.distances == None:
            self.distances = [[[0 for x in range(4)] for x in range(self.grid.height)] for x in range(self.grid.width)]

        for x in range(0, self.grid.width):
            for y in range(0, self.grid.height):
                if self.grid.cell_at(x, y) == 100: continue     # continue if wall

                self.distances[x][y][0] = 0
                for y_2 in range(y + 1, self.grid.height):
                    if self.grid.cell_at(x, y_2) == 100: break  # stop counting if we hit a wall
                    self.distances[x][y][0] += self.grid.resolution

                self.distances[x][y][2] = 0
                for y_2 in range(y - 1, -1, -1):
                    if self.grid.cell_at(x, y_2) == 100: break  # stop counting if we hit a wall
                    self.distances[x][y][2] += self.grid.resolution

                self.distances[x][y][3] = 0
                for x_2 in range(x + 1, self.grid.width):
                    if self.grid.cell_at(x_2, y) == 100: break
                    self.distances[x][y][3] += self.grid.resolution

                self.distances[x][y][1] = 0
                for x_2 in range(x - 1, -1, -1):
                    if self.grid.cell_at(x_2, y) == 100: break
                    self.distances[x][y][1] += self.grid.resolution

    def laser_callback(self, msg):
        if self.state != fsm.SCAN: return
        print("begin laser")

        min_index = int((MIN_SCAN_ANGLE_RAD - msg.angle_min) / msg.angle_increment)
        max_index = int((MAX_SCAN_ANGLE_RAD - msg.angle_min) / msg.angle_increment)

        if self.possible_positions == None:
            # if no possible positions have been computed yet, go over all grid squares and every posssible orientation

            self.possible_positions = []
            for forward_angle_index in range(min_index, max_index + 1):
                # angles and indices of those angles in the order [forward, left, back, right]
                angles = [forward_angle_index * msg.angle_increment + msg.angle_min, 0, 0, 0]
                indices = [forward_angle_index, 0, 0, 0]

                for i in range(1, 4):
                    angles[i] = angles[0] + i * math.pi / 2
                    if angles[i] > math.pi: angles[i] -= 2 * math.pi
                    indices[i] = int((angles[i] - msg.angle_min) / msg.angle_increment)

                for x in range(0, self.grid.width):
                    for y in range(0, self.grid.height):
                        sum_squared_errors = 0
                        for i in range(4):
                            sum_squared_errors += (msg.ranges[indices[i]] - self.distances[x][y][i]) ** 2
                        if sum_squared_errors <= self.error_threshold:
                            self.possible_positions.append(Position(x, y, -angles[0]))
        else:
            # if possible positions have been previously computed, only choose among those
            for position in self.possible_positions:
                # calculate new position robot must be in based on last rotation/translation
                new_x = position.x + math.cos(position.theta) * self.last_translation
                new_y = position.y + math.sin(position.theta) * self.last_translation
                new_theta = position.theta + self.last_rotation
                if new_theta > math.pi: new_theta -= 2 * math.pi

                # calculate forward, left, rear, and back scan angles
                angles = [-new_theta, 0, 0, 0]
                for i in range(1, 4):
                    angles[i] = angles[0] + i * math.pi / 2
                    if angles[i] > math.pi: angles[i] -= 2 * math.pi

                # calculate indices for corresponding scan angles
                indices = []
                for i in range(4):
                    indices.append(int((angles[i] - msg.angle_min) / msg.angle_increment))

                # calculate error based on scans and theoretical grid distances
                sum_squared_errors = 0
                for i in range(4):
                    sum_squared_errors += (msg.ranges[indices[i]] - self.distances[new_x][new_y][i]) ** 2

                # remove possible positions if error is above threshold
                if sum_squared_errors > self.error_threshold:
                    self.possible_positions.remove(position)

        print("possible positions: ")
        for position in self.possible_positions:
            print("x: {}, y: {}, theta: {}".format(position.x, position.y, position.theta))
        # stop if localized / no possible positions
        if len(self.possible_positions) <= 1:
            self.state = fsm.STOP
            return

        # else, move the robot: translate if there is space ahead, else rotate to make space
        self.state = fsm.MOVE
        if msg.ranges[int((-msg.angle_min) / msg.angle_increment)] > self.grid.resolution:
            self.last_translation = self.grid.resolution
            self.last_rotation = 0
        else:
            self.last_translation = 0
            self.last_rotation = math.pi / 2
        self.move(self.last_translation, self.last_rotation)
        self.state = fsm.SCAN

    def move(self, translation, rotation):
        """
        Moves the robot forward by the given distance (translation in metres)
        and also rotates it by the given angle (in radian)
        """
        # find required duration for both translation and duration, and choose
        # max of both for final duration
        translation_duration = abs(translation / self.linear_velocity)
        rotation_duration = abs(rotation / self.angular_velocity)
        final_duration = max(translation_duration, rotation_duration)
        # determine linear and angular velocities based on duration; one may
        # have to be lowered to match the movement requirements
        lin_v = self.linear_velocity
        ang_v = self.angular_velocity
        if translation_duration < final_duration:
            lin_v = translation / final_duration
        elif rotation_duration < final_duration:
            ang_v = rotation / final_duration
        # determine sign of angular velocity
        if rotation * ang_v < 0:
            ang_v = -1 * ang_v
        # movement
        rate = rospy.Rate(FREQUENCY)
        start_time = rospy.get_rostime()
        while not rospy.is_shutdown():
            # check if done
            if rospy.get_rostime() - start_time >= rospy.Duration(final_duration):
                break
            twist_msg = Twist()
            twist_msg.linear.x = lin_v
            twist_msg.angular.z = ang_v
            self.cmd_pub.publish(twist_msg)
            rate.sleep()    # sleep to maintain given frequency

    def spin(self):
        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():            
            rate.sleep()

def main():
    """Main function."""
    rospy.init_node("localizer")
    rospy.sleep(2)
    localizer = Localizer()
    try:
        localizer.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

if __name__ == "__main__":
    main()
