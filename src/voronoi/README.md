# Graph Server

The graph server takes an occupancy grid from file and does two things:

1. Publishes the map as OccupancyGrid messages on the `/map` topic (configurable)
2. Runs a ROS service that will return a Voronoi Graph in JSON format parsed from the occupancy grid

## Message Format

The message contents are a JSON string of each node's position, id, and neighboring nodes.
```json
[
  {
    "x": 0.0,
    "y": 5.0,
    "id": 0,
    "neighbor_ids": [1, 2]
  },
  { 
    "x": 10.2,
    "y": 3.1,
    "id": 1,
    "neighbor_ids": [0]
  },
  { 
    "x": 5.0,
    "y": 5.0,
    "id": 2,
    "neighbor_ids": [0]
  }
]
```

