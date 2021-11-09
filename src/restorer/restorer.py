import rospy  # module for ROS APIs
from geometry_msgs.msg import Twist  # message type for velocity command.


class Restorer:
    """When the robot has finished seeking the target, the restorer kicks in and moves the robot back to
    the graph.
    """

    def __init__(self, is_live):
        """Initialization function."""

    def main(self):
        """Loops; triggers """


if __name__ == "__main__":
    # 1st. initialization of node.
    rospy.init_node("restorer")

    # 2nd. Creation of the class with relevant publishers/subscribers.
    restorer = Restorer(is_live=False)

    # 3rd loop.
    restorer.main()
