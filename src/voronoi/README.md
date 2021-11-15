# Graph Server

The graph server takes an occupancy grid from file and runs a ROS service that will return a Voronoi Graph in JSON format parsed from a published occupancy grid.

## Requierments

* numpy
* scipy
* scikit-image
* ROS

## Running the Server

0. Ensure a valid occupancy grid is being published and make sure its topic matches the global varibale `DEFAUL_MAP_TOPIC`
1. Run the executable: `./graph_server.py
2. Have some external code call its ROS service (or manually: `rosservice call /graph "{}"`
3. Profit!

## Message Format

The message contents are a JSON string of each node's position, id, and neighboring nodes.
```json
[
  {
    "x": 0.0,
    "y": 5.0,
    "id": 0,
    "neighbors": [1, 2]
  },
  { 
    "x": 10.2,
    "y": 3.1,
    "id": 1,
    "neighbors": [0]
  },
  { 
    "x": 5.0,
    "y": 5.0,
    "id": 2,
    "neighbors": [0]
  }
]
```
