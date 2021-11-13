def get_test_json_graph():
    """Gets a test JSON graph to use for evaluating the patroller"""
    return '[' \
           '{"x": 1.0, "y": 0.0, "id": 0, "neighbor_ids": [1]},' \
           '{"x": 1.0, "y": 1.0, "id": 1, "neighbor_ids": [0, 2]},' \
           '{"x": 2.0, "y": 1.0, "id": 2, "neighbor_ids": [1, 3, 7]},' \
           '{"x": 6.0, "y": 1.0, "id": 3, "neighbor_ids": [2, 8, 4]},' \
           '{"x": 6.0, "y": 3.0, "id": 4, "neighbor_ids": [3, 9, 5]},' \
           '{"x": 6.0, "y": 5.0, "id": 5, "neighbor_ids": [4, 6]},' \
           '{"x": 2.0, "y": 5.0, "id": 6, "neighbor_ids": [5, 8, 7]},' \
           '{"x": 2.0, "y": 3.0, "id": 7, "neighbor_ids": [6, 8, 2]},' \
           '{"x": 4.0, "y": 3.0, "id": 8, "neighbor_ids": [3, 7, 6]},' \
           '{"x": 7.0, "y": 3.0, "id": 9, "neighbor_ids": [10, 4, 13]},' \
           '{"x": 7.0, "y": 2.0, "id": 10, "neighbor_ids": [9, 11]},' \
           '{"x": 8.0, "y": 2.0, "id": 11, "neighbor_ids": [10, 12]},' \
           '{"x": 8.0, "y": 4.0, "id": 12, "neighbor_ids": [11, 13]},' \
           '{"x": 7.0, "y": 3.0, "id": 13, "neighbor_ids": [12, 9]}' \
           ']'


def get_test_nodes_to_visit():
    """Gets a list of the nodes to visit"""
    return ([
           {"x": 0.0, "y": 0.0},
           {"x": 1.0, "y": 0.0},
           {"x": 2.0, "y": 1.0},
           {"x": 2.0, "y": 2.0}
    ])

