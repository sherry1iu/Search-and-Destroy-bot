import rospy  # module for ROS APIs
from geometry_msgs.msg import Twist  # message type for velocity command.
from std_msgs.msg import String # message type for rotation_warning

FREQUENCY = 10

class Patroller:
    """Patrols the graph, looking for objects to pursue
    """

    def __init__(self, is_live):
        """Initialization function."""
        self.mode = "initializing"
        self.is_on_graph = False

        # callback and publisher for mode switching
        self.mode_callback = rospy.Subscriber("mode", String, self.mode_callback, queue_size=1)
        self.mode_publisher = rospy.Subscriber("mode", String, queue_size=1)

        # whether we should re-do the planned patrol route
        self.should_plan = False

        # the JSON string containing the graph; we get this from the graph solving service
        self.raw_graph_string = None

        # the nodes in the graph
        self.node_dictionary = None
        # the edges in the graph
        self.edge_dictionary = None
        # a queue of nodes in the graph to visit
        self.nodes_to_visit = []

    def mode_callback(self, msg):
        """The callback for switching modes"""
        if msg.data is "patrolling":
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

            rate.sleep()

if __name__ == "__main__":
    # 1st. initialization of node.
    rospy.init_node("patroller")

    # 2nd. Creation of the class with relevant publishers/subscribers.
    patroller = Patroller(is_live=False)

    # 3rd loop.
    patroller.main()
