def get_test_json_graph():
    """Gets a test JSON graph to use for evaluating the patroller"""
    return '[' \
           '{"x": 1.0, "y": 0.0, "id": 0, "neighbor_ids": [1, 2]},' \
           '{"x": 2.0, "y": 1.0, "id": 1, "neighbor_ids": [0, 2, 3]},' \
           '{"x": 1.0, "y": 1.0, "id": 2, "neighbor_ids": [0, 1]}' \
           '{"x": 2.0, "y": 2.0, "id": 3, "neighbor_ids": [1]}}' \
           ']'


def get_test_nodes_to_visit():
    """Gets a list of the nodes to visit"""
    return ([
           {"x": 0.0, "y": 0.0},
           {"x": 1.0, "y": 0.0},
           {"x": 2.0, "y": 1.0},
           {"x": 2.0, "y": 2.0}
    ])

