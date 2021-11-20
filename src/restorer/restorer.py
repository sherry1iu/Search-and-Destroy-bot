#!/usr/bin/env python

# Author: Elliot Potter, CS 81, Dartmouth College
# Date: 11/11/2021

import sys
sys.path.append('../../')

import numpy
import rospy  # module for ROS APIs
from geometry_msgs.msg import Twist  # message type for velocity command.
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String # message type for rotation_warning

from src.test_info.get_test_json_graph import get_test_json_graph
import tf

from src.restorer.bfs import BFS
from src.utilities.get_current_position import get_current_position
from src.utilities.move_to_point import move_to_point
from src.utilities.parse_graph import parse_json_graph

from std_srvs.srv import Trigger, TriggerRequest

FREQUENCY = 10
OCCUPANCY_THRESHOLD = 0.75


class Restorer:
    """When the robot has finished seeking the target, the restorer kicks in and moves the robot back to
    the graph.
    """

    def __init__(self, is_live, is_test_mode):
        """Initialization function."""
        if is_test_mode:
            self.mode = "restoring"
            self.raw_graph_string = get_test_json_graph()
            # NOTE -- if in testing, also run ../utilities/tf_map_publisher

        else:
            # self.mode = "initializing"
            self.mode = "restoring"
            self._graph_service_proxy = rospy.ServiceProxy("graph", Trigger)
            self.raw_graph_string = None

        # callback and publisher for mode switching
        self.mode_callback = rospy.Subscriber("mode", String, self.mode_callback, queue_size=1)
        self.mode_publisher = rospy.Publisher("mode", String, queue_size=1)

        # Callback for the occupancy grid
        self.grid_subscriber = rospy.Subscriber("blur_map", OccupancyGrid, self.grid_callback)
        # this will be populated with the data from the grid
        self.grid_msg = None
        # this is the grid converted into a double-nested array
        self.grid_arrays = None

        self.move_cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.transform_listener = tf.TransformListener()

        # the nodes in the graph
        self.node_dictionary = None
        # the edges in the graph
        self.edge_dictionary = None

        self.should_plan = True
        # a queue of points to get back to the graph
        self.path_points_to_visit = []

        rospy.sleep(2)

        # if in test mode, we will just outright call the parsing function
        if not is_test_mode:
            # the JSON string containing the graph; we get this from the graph solving service
            self.raw_graph_string = self._graph_service_proxy(TriggerRequest()).message

        # use the raw graph string to create node and edge dictionaries
        self.node_dictionary, self.edge_dictionary = parse_json_graph(self.raw_graph_string)

    def raw_graph_callback(self, raw_graph_string):
        """Callback for parsing the raw graph created by another node"""
        self.node_dictionary, self.edge_dictionary = parse_json_graph(raw_graph_string.message)

    def mode_callback(self, msg):
        """The callback for switching modes"""
        print("New mode is: " + msg.data)
        if msg.data is "restoring" and self.mode is not "restoring":
            # we will want to re-plan, because we may be at a new location on the graph
            print("We want to re-plan")
            self.should_plan = True
        self.mode = msg.data

    def publish_mode(self, mode):
        """The publisher for switching modes"""
        self.mode = mode
        msg = String()
        msg.data = mode
        self.mode_publisher.publish(msg)

    def grid_callback(self, grid_msg):
        """The callback to set the grid instance variable"""
        self.grid_msg = grid_msg
        # converts grid_msg.data into a double-nested array
        self.grid_arrays = numpy.reshape(grid_msg.data, (grid_msg.info.height, grid_msg.info.width))

    def plan(self):
        """Makes a plan to get the robot back onto the graph"""
        if self.grid_arrays is None:
            return

        print("Planning route back to the graph")

        trans, rot = get_current_position("map", "base_link", self.transform_listener)
        node_array = []

        for key in self.node_dictionary:
            node_array.append({
                "x": int(float(self.node_dictionary[key]["x"] - self.grid_msg.info.origin.position.x) / self.grid_msg.info.resolution),
                "y": int(float(self.node_dictionary[key]["y"] - self.grid_msg.info.origin.position.y) / self.grid_msg.info.resolution)
            })

        bfs = BFS(
            {
                "x": int(float(trans[0] - self.grid_msg.info.origin.position.x) / self.grid_msg.info.resolution),
                "y": int(float(trans[1] - self.grid_msg.info.origin.position.y) / self.grid_msg.info.resolution)
            },
            node_array,
            self.grid_arrays
        )

        path_points_to_visit = bfs.perform_search(OCCUPANCY_THRESHOLD)

        # Restores the x-y resolution, rather than the grid resolution
        for i in range(len(path_points_to_visit)):
            path_points_to_visit[i] = {
                "x": path_points_to_visit[i]["x"] * self.grid_msg.info.resolution + self.grid_msg.info.origin.position.x,
                "y": path_points_to_visit[i]["y"] * self.grid_msg.info.resolution + self.grid_msg.info.origin.position.y,
            }

        print(path_points_to_visit)

        self.path_points_to_visit = path_points_to_visit
        self.should_plan = False

    def restore(self):
        """Executes the plan for returning to the graph"""
        print("Restoring back to the graph")

        if len(self.path_points_to_visit) == 0:
            # if we've run out of points, we are now on the graph
            self.publish_mode("patrolling")
            print("Patrolling now...")
            return
            # self.plan()

        next_node = self.path_points_to_visit.pop(0)
        trans, rot = get_current_position("map", "base_link", self.transform_listener)
        yaw = tf.transformations.euler_from_quaternion(rot)[2]

        # print("Next node:")
        # print(next_node)
        # print("Current node:")
        # print(trans)
        # print("Orientation:")
        # print(yaw)
        # print("\n")

        move_to_point(current_point=trans,
                      current_orientation=yaw,
                      desired_point=[next_node["x"], next_node["y"]],
                      move_cmd_pub=self.move_cmd_pub,
                      transform_listener=self.transform_listener
                      )

    def main(self):
        """Loops; triggers BFS to restore the robot to one of the nodes on the graph"""
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
        while not rospy.is_shutdown():
            if self.mode == "restoring":
                if self.should_plan:
                    print("Planning restoration to the graph...")
                    self.plan()
                else:
                    print("Restoring the robot to the graph")
                    self.restore()

            rate.sleep()

if __name__ == "__main__":
    # 1st. initialization of node.
    rospy.init_node("restorer")

    # 2nd. Creation of the class with relevant publishers/subscribers.
    restorer = Restorer(is_live=False, is_test_mode=False)

    # 3rd loop.
    restorer.main()
