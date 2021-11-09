import rospy  # module for ROS APIs
from geometry_msgs.msg import Twist  # message type for velocity command.

class Restorer:
    """When the robot has finished seeking the target, the restorer kicks in and moves the robot back to
    the graph.
    """

    def __init__(self, is_live):
        """Initialization function."""