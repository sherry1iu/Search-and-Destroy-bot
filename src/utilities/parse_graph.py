import json
import math


def parse_json_graph(json_graph):
    """
    This function parses a JSON graph and returns the representation in data that we're interested in.
    The expected input format is:
    [{"x": 3.0, "y": 4.0, "id": 7, "neighbor_ids": [8, 10]}]

    the output is:
    # the node dictionary
    {7: {"x": 3.0, "y": 4.0} },
    # the nested edge dictionary (containing the Euclidean distance between nodes)
    {
        7: { 8: 12.5, 10: 5.3 }
    }
    """

    simple_graph = json.loads(json_graph)
    node_dictionary = {}
    edge_dictionary = {}

    # loop over nodes the first time
    for node in simple_graph:
        node_dictionary[node["id"]] = {"x": node["x"], "y": node["y"]}

    # loop over nodes the second time
    for node in simple_graph:
        # create an inner dictionary for this node
        edge_sub_dictionary = {}
        for neighbor_id in node["neighbors"]:
            neighbor = node_dictionary[neighbor_id]
            difference = [neighbor["x"] - node["x"],
                          neighbor["y"] - node["y"]]

            # find the length between nodes
            length = math.sqrt(difference[0] ** 2 + difference[1] ** 2)
            # updates the inner edge dictionary
            edge_sub_dictionary[neighbor_id] = length

        # updates edge dictionary
        edge_dictionary[node["id"]] = edge_sub_dictionary

    # returns both the node dictionary and the edge dictionary
    return node_dictionary, edge_dictionary
