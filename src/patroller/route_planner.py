import sys
sys.path.append('../../')

from src.test_info.get_test_json_graph import get_test_nodes_to_visit
from src.utilities.move_to_point import get_distance_angle_between_points


class RoutePlanner:
    """A class designed to use the Chinese Postman Problem to list the nodes the robot must visit"""

    def __init__(self, node_dictionary, edge_dictionary, current_location):
        self.node_dictionary = node_dictionary
        self.edge_dictionary = edge_dictionary
        self.current_node = self.find_current_node(current_location)

    def find_current_node(self, current_location):
        """We assume that we're basically sitting on top of a node -- all we need to do is find which one"""
        closest_node = None
        closest_node_distance = None

        temp_current_loc = [current_location["x"], current_location["y"]]

        for _id in self.node_dictionary:
            temp_target_point = [self.node_dictionary[_id]["x"], self.node_dictionary[_id]["y"]]
            length, angle_to_rotate, target_angle = get_distance_angle_between_points(
                temp_target_point,
                temp_current_loc,
                0
            )
            if closest_node is None or length < closest_node_distance:
                closest_node_distance = length
                closest_node = self.node_dictionary[_id]

        return closest_node

    def find_traversal(self):
        """Finds an efficient traversal of the graph such that all edges are traversed at least once"""
        return get_test_nodes_to_visit()
