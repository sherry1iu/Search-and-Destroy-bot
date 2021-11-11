import numpy
from nav_msgs.msg import OccupancyGrid

GRID_WIDTH = 10
GRID_HEIGHT = 10
GRID_RESOLUTION = 0.05
GRID_ORIGIN = {"x": 0, "y": 0}  # w.r.t the map reference frame


def get_test_grid():
    """Gets an OccupancyGrid for testing purposes"""
    map_msg = OccupancyGrid()
    map_msg.info.resolution = GRID_RESOLUTION
    map_msg.info.width = int(GRID_WIDTH / GRID_RESOLUTION)
    map_msg.info.height = int(GRID_HEIGHT / GRID_RESOLUTION)
    map_msg.info.origin.position.x = GRID_ORIGIN["x"]
    map_msg.info.origin.position.y = GRID_ORIGIN["y"]

    map_msg.data = numpy.zeros((map_msg.info.height, map_msg.info.width))

    return map_msg

