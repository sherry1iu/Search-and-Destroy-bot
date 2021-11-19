#!/usr/bin/env python

# Localizer module
# Uses laser scanner and occupancy grid to localize robot

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
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
DEFAULT_SCAN_TOPIC = 'scan'
ODOM_TOPIC = 'odom'
GRID_TOPIC = 'static_map'

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
        self.mode = "initializing"

        self.grid = None
        self.distances = None
        self.possible_positions = None
        self.error_threshold = None

        self.last_translation = 0
        self.last_rotation = 0

        self.linear_velocity = 0.1
        self.angular_velocity = math.pi / 4

        self.grid_sub = rospy.Subscriber(GRID_TOPIC, OccupancyGrid, self.grid_callback, queue_size=1)
        self.laser_sub = rospy.Subscriber(DEFAULT_SCAN_TOPIC, LaserScan, self.laser_callback, queue_size=1)
        self.cmd_pub = rospy.Publisher(DEFAULT_CMD_VEL_TOPIC, Twist, queue_size=1)
        self.pose_pub = rospy.Publisher("amcl_pose", PoseWithCovarianceStamped, queue_size=1)

        self.mode_callback = rospy.Subscriber("mode", String, self.mode_callback, queue_size=1)
        self.mode_publisher = rospy.Publisher("mode", String, queue_size=1)

        self.br = tf.TransformBroadcaster()

        self.final_pos = None

        print("Ready to localize")

    def mode_callback(self, msg):
        print(msg)
        """The callback for switching modes"""
        if msg.data is "initializing" and self.mode is not "initializing":
            self.state = fsm.UNINIT
        self.mode = msg.data

    def mode_publisher(self, mode):
        """The publisher for switching modes"""
        self.mode = mode
        msg = String()
        msg.data = mode
        self.mode_publisher.publish(msg)

    def grid_callback(self, msg):
        if self.mode is not "initializing" or self.state != fsm.UNINIT: return
        self.grid = Grid(msg.data, msg.info.width, msg.info.height, msg.info.resolution)
        print("created grid")
        self.error_threshold = (30 * self.grid.resolution) ** 2
        self.build_distances()
        self.state = fsm.SCAN
        print(self.state)

    def build_distances(self):
        """Builds a dictionary of forward/back/left/right distances from every grid square"""
        if self.distances == None:
            self.distances = [[[0 for x in range(4)] for x in range(self.grid.height)] for x in range(self.grid.width)]

        # left distances
        distance_to_wall = 0
        for y in range(self.grid.height):
            for x in range(self.grid.width):
                self.distances[x][y][1] = distance_to_wall
                # if wall, reset distance counter
                if self.grid.cell_at(x, y) == 100:
                    distance_to_wall = 0
                else:
                    distance_to_wall += self.grid.resolution

        # right distances
        distance_to_wall = 0
        for y in range(self.grid.height):
            for x in range(self.grid.width - 1, -1, -1):
                self.distances[x][y][3] = distance_to_wall
                # if wall, reset distance counter
                if self.grid.cell_at(x, y) == 100:
                    distance_to_wall = 0
                else:
                    distance_to_wall += self.grid.resolution

        # forward distances
        distance_to_wall = 0
        for x in range(self.grid.width):
            for y in range(self.grid.height):
                self.distances[x][y][0] = distance_to_wall
                # if wall, reset distance counter
                if self.grid.cell_at(x, y) == 100:
                    distance_to_wall = 0
                else:
                    distance_to_wall += self.grid.resolution

        # back distances
        distance_to_wall = 0
        for x in range(self.grid.width):
            for y in range(self.grid.height - 1, -1, -1):
                self.distances[x][y][2] = distance_to_wall
                # if wall, reset distance counter
                if self.grid.cell_at(x, y) == 100:
                    distance_to_wall = 0
                else:
                    distance_to_wall += self.grid.resolution

        print("built distances")

    def correct_angle(self, angle):
        corrected_angle = angle
        if angle > math.pi:
            corrected_angle -= 2 * math.pi
        elif angle < -math.pi:
            corrected_angle += 2 * math.pi
        return corrected_angle

    def forward_to_orientation(self, forward_angle):
        """Converts given forward angle to robot's angle of orientation"""
        if forward_angle < 0:
            return -(math.pi / 2 + forward_angle)
        if forward_angle <= math.pi / 2:
            return math.pi / 2 - forward_angle
        return 3 / 2 * math.pi - forward_angle

    def orientation_to_forward(self, orientation):
        """Converts given forward angle to robot's angle of orientation"""
        if abs(orientation) <= math.pi / 2:
            return -(math.pi / 2 + orientation)
        if orientation < 0:
            return math.pi / 2 - forward_angle
        return 3 / 2 * math.pi - forward_angle

    def laser_callback(self, msg):
        # print(self.state)
        if self.mode is not "initializing" or self.state != fsm.SCAN: return
        print("begin laser")

        min_index = int((MIN_SCAN_ANGLE_RAD - msg.angle_min) / msg.angle_increment)
        max_index = int((MAX_SCAN_ANGLE_RAD - msg.angle_min) / msg.angle_increment)

        if self.possible_positions == None:
            # if no possible positions have been computed yet, go over all grid squares and every posssible orientation

            self.possible_positions = []
            for forward_angle_index in range(min_index, max_index + 1, (max_index + 1 - min_index) / 36):
                # angles and indices of those angles in the order [forward, left, back, right]
                angles = [forward_angle_index * msg.angle_increment + msg.angle_min, 0, 0, 0]
                indices = [forward_angle_index, 0, 0, 0]

                for i in range(1, 4):
                    angles[i] = angles[0] + i * math.pi / 2
                    if angles[i] > math.pi: angles[i] -= 2 * math.pi
                    indices[i] = int((angles[i] - msg.angle_min) / msg.angle_increment)

                for x in range(0, self.grid.width, 10):
                    for y in range(0, self.grid.height, 10):
                        sum_squared_errors = 0
                        for i in range(4):
                            sum_squared_errors += (msg.ranges[indices[i]] - self.distances[x][y][i]) ** 2
                        if sum_squared_errors <= self.error_threshold:
                            self.possible_positions.append(Position(x, y, self.correct_angle(self.forward_to_orientation(angles[0]))))
        else:
            # if possible positions have been previously computed, only choose among those
            for pos_i, position in enumerate(self.possible_positions):
                # calculate new position robot must be in based on last rotation/translation
                new_x = int(position.x + math.cos(position.theta) * self.last_translation)
                new_y = int(position.y + math.sin(position.theta) * self.last_translation)
                new_theta = self.correct_angle(position.theta + self.last_rotation)
                new_fwd_angle = self.correct_angle(self.orientation_to_forward(new_theta))

                # calculate forward, left, rear, and back scan angles
                angles = [new_fwd_angle, 0, 0, 0]
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
                else:
                    # update new position
                    self.possible_positions[pos_i].x = new_x
                    self.possible_positions[pos_i].y = new_y
                    self.possible_positions[pos_i].theta = new_theta

        print("possible positions: ")
        for position in self.possible_positions:
            print("x: {}, y: {}, theta: {}".format(position.x, position.y, position.theta))

        # stop if localized / no possible positions
        if len(self.possible_positions) <= 2:
            self.state = fsm.STOP
            self.final_pose = Position(self.possible_positions[0].x * self.grid.resolution, self.possible_positions[0].y * self.grid.resolution, self.possible_positions[0].theta)
            self.mode_publisher("restoring")
            return

        # else, move the robot: translate if there is space ahead, else rotate to make space
        self.state = fsm.MOVE
        if msg.ranges[int((-msg.angle_min) / msg.angle_increment)] > self.grid.resolution:
            self.last_translation = self.grid.resolution
            self.last_rotation = 0
        else:
            self.last_translation = 0
            self.last_rotation = random.random() * math.pi / 2
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
                twist_msg = Twist()
                twist_msg.linear.x = 0
                twist_msg.angular.z = 0
                self.cmd_pub.publish(twist_msg)
                break
            twist_msg = Twist()
            twist_msg.linear.x = lin_v
            twist_msg.angular.z = ang_v
            self.cmd_pub.publish(twist_msg)
            rate.sleep()    # sleep to maintain given frequency

    def publish_final_pose(self, pos):
        msg = PoseWithCovarianceStamped()
        
        msg.header.seq = 0
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"

        msg.pose.pose.position.x = pos.x
        msg.pose.pose.position.x = pos.y

        quaternion = tf.transformations.quaternion_from_euler(0, 0, pos.theta)
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]


        self.br.sendTransform((pos.x, pos.y, 0),
                         tf.transformations.quaternion_from_euler(0, 0, pos.theta),
                         rospy.Time.now(),
                         "map",
                         DEFAULT_SCAN_TOPIC
                         )

        self.pose_pub.publish(msg)

    def spin(self):
        rate = rospy.Rate(FREQUENCY)
        while not rospy.is_shutdown():
            if self.state == fsm.STOP and self.final_pos != None:
                self.publish_final_pose(self.final_pos)
            rate.sleep()

def main():
    """Main function."""
    rospy.init_node("localizer")
    localizer = Localizer()
    rospy.sleep(2)

    try:
        localizer.spin()
    except rospy.ROSInterruptException:
        rospy.logerr("ROS node interrupted.")

if __name__ == "__main__":
    main()
