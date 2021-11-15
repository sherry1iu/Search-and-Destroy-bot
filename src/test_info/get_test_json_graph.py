def get_test_json_graph():
    """Gets a test JSON graph to use for evaluating the patroller"""
    return '[' \
           '{"x": 1.0, "y": 0.0, "id": 0, "neighbors": [1]},' \
           '{"x": 1.0, "y": 1.0, "id": 1, "neighbors": [0, 2]},' \
           '{"x": 2.0, "y": 1.0, "id": 2, "neighbors": [1, 3, 7]},' \
           '{"x": 6.0, "y": 1.0, "id": 3, "neighbors": [2, 8, 4]},' \
           '{"x": 6.0, "y": 3.0, "id": 4, "neighbors": [3, 9, 5]},' \
           '{"x": 6.0, "y": 5.0, "id": 5, "neighbors": [4, 6]},' \
           '{"x": 2.0, "y": 5.0, "id": 6, "neighbors": [5, 8, 7]},' \
           '{"x": 2.0, "y": 3.0, "id": 7, "neighbors": [6, 8, 2]},' \
           '{"x": 4.0, "y": 3.0, "id": 8, "neighbors": [3, 7, 6]},' \
           '{"x": 7.0, "y": 3.0, "id": 9, "neighbors": [10, 4, 13]},' \
           '{"x": 7.0, "y": 2.0, "id": 10, "neighbors": [9, 11]},' \
           '{"x": 8.0, "y": 2.0, "id": 11, "neighbors": [10, 12]},' \
           '{"x": 8.0, "y": 4.0, "id": 12, "neighbors": [11, 13]},' \
           '{"x": 7.0, "y": 4.0, "id": 13, "neighbors": [12, 9]}' \
           ']'


def get_test_nodes_to_visit():
    """Gets a list of the nodes to visit"""
    return ([
           {"x": 0.0, "y": 0.0},
           {"x": 1.0, "y": 0.0},
           {"x": 2.0, "y": 1.0},
           {"x": 2.0, "y": 2.0}
    ])

