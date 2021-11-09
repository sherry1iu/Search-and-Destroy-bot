#!/usr/bin/env python

# Author: Elliot Potter, CS 81, Dartmouth College
# Date: 11/9/2021

import sys
sys.path.append('../../')

import rospy  # module for ROS APIs
from geometry_msgs.msg import Twist  # message type for velocity command.
from std_msgs.msg import String # message type for rotation_warning
import tf  # library for transformations.

from src.patroller.get_test_json_graph import get_test_json_graph
from src.patroller.route_planner import RoutePlanner
from src.utilities.get_current_position import get_current_position_map, translate_point_between_frames
from src.utilities.move_to_point import move_to_point
from src.utilities.parse_graph import parse_json_graph

FREQUENCY = 10


class Patroller:
    """Patrols the graph, looking for objects to pursue
    """

    def __init__(self, is_live, is_test_mode):
        """Initialization function."""
        if is_test_mode:
            self.mode = "patrolling"
            self.is_on_graph = True
            self.raw_graph_string = get_test_json_graph()

        else:
            self.mode = "initializing"
            self.is_on_graph = False

            # the JSON string containing the graph; we get this from the graph solving service
            self.raw_graph_string = None

        # callback and publisher for mode switching
        self.mode_callback = rospy.Subscriber("mode", String, self.mode_callback, queue_size=1)
        self.mode_publisher = rospy.Subscriber("mode", String, queue_size=1)

        # whether we should re-do the planned patrol route
        self.should_plan = True

        self.move_cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.transform_listener = tf.TransformListener()

        # the nodes in the graph
        self.node_dictionary = None
        # the edges in the graph
        self.edge_dictionary = None
        # a queue of nodes in the graph to visit
        self.nodes_to_visit = []

        # if in test mode, we will just outright call the parsing function
        if is_test_mode:
            parse_json_graph(self.raw_graph_string)

    def mode_callback(self, msg):
        """The callback for switching modes"""
        if msg.data is "patrolling" and self.mode is not "patrolling":
            # we will want to re-plan, because we may be at a new location on the graph
            self.should_plan = True
        self.mode = msg.data

    def mode_publisher(self, mode):
        """The publisher for switching modes"""
        self.mode = mode
        msg = String()
        msg.data = mode
        self.mode_publisher.publish(msg)

    def plan(self):
        """Makes a plan for traversing the graph"""
        trans = get_current_position_map(self.transform_listener)
        planner = RoutePlanner(
            node_dictionary=self.node_dictionary,
            edge_dictionary=self.edge_dictionary,
            current_location={"x": trans.x, "y": trans.y}
        )
        self.nodes_to_visit = planner.find_traversal()

    def execute_plan(self):
        """Executes the plan for traversing the graph"""
        if len(self.nodes_to_visit) is 0:
            # re-plan if we just traversed the entire graph
            self.plan()

        next_node = self.nodes_to_visit.pop(0)
        next_node_in_base_link = translate_point_between_frames(
            point=next_node,
            frame1="map",
            frame2="base_link",
            transform_listener=self.transform_listener
        )
        move_to_point(current_point={"x": 0, "y": 0},
                      current_orientation=0,
                      desired_point=next_node_in_base_link,
                      move_cmd_pub=self.move_cmd_pub
                      )

    def main(self):
        """Loops; triggers patrolling if mode is patrolling"""
        rate = rospy.Rate(FREQUENCY) # loop at 10 Hz.
        while not rospy.is_shutdown():
            if self.mode is "patrolling":
                # if we haven't arrived on the graph yet, we will use the Restorer to get us there
                if not self.is_on_graph:
                    self.mode_publisher("restoring")
                    self.is_on_graph = True

                if self.should_plan is True:
                    self.plan()
                else:
                    self.execute_plan()

            rate.sleep()

if __name__ == "__main__":
    # 1st. initialization of node.
    rospy.init_node("patroller")

    # 2nd. Creation of the class with relevant publishers/subscribers.
    patroller = Patroller(is_live=False, is_test_mode=True)

    # 3rd loop.
    patroller.main()
