#!/usr/bin/env python

# A ROS service that generates a topological map of a published occupancy grid
# Author: Isaac Feldman, COSC 81 Fall 2021
import json
from collections import deque
import numpy as np

# Image processing Imports
from scipy import ndimage
from skimage import morphology
from skimage.feature import corner_harris, corner_peaks

# ROS Imports
import rospy
from nav_msgs.msg import OccupancyGrid          # http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
from std_srvs.srv import Trigger, TriggerResponse

DEFAULT_MAP_TOPIC    = "map"
DEFAULT_SERVICE      = "graph"
CORNER_SENS          = 0.025
THIN                 = 0.5

class Server():
    def __init__(self):
        self._map = None
        self._map_resolution = None
        self._skel = None
        self._count = 0

        self._graph_service = rospy.Service(DEFAULT_SERVICE, Trigger, self._graph_callback)
        self._map_sub = rospy.Subscriber(DEFAULT_MAP_TOPIC, OccupancyGrid, self._map_callback, queue_size=1)

    def _map_callback(self, msg):
        """ Process the raw occupancy grid message into a useful map"""
        width = int(msg.info.width)
        height = int(msg.info.height)
        self._map_resolution = float(msg.info.resolution)

        grid = np.reshape(msg.data, (height, width))
        self._map = grid

    def _graph_callback(self, req):
        """
        Process the map into a graph by request
        """
        resp = TriggerResponse()
        response_graph = json.dumps(self.compute_graph())
        resp.success = True
        resp.message = response_graph 
        return resp

    def compute_graph(self):
        """ Compute a topological graph from the provided map """
        while self._map is None:
            pass # block until we have a map

        d, f = ndimage.distance_transform_edt(self._map, return_indices=True)
        mean = np.mean(d)
        self._skel = morphology.skeletonize(d > mean*THIN)
        corners = corner_peaks(corner_harris(self._skel, k=CORNER_SENS), min_distance=1)

        coords = []
        for c in corners:
          coords.append((c[1], c[0])) #x, y

        # First do a few traversal to find the neighboring feature nodes

        graph = self._add_nodes(coords)
        for i in range(10): # do this a few times
          self._add_neighbors(graph, coords)

        # Swap out the positions for the proper ids
        ids = {}
        for node in graph:
          ids[(node["x"],node["y"])] = node["id"]

        for node in graph:
          new_neighbors = []
          for n in node["neighbors"]:
            new_neighbors.append(ids[n])
          node["neighbors"] = new_neighbors

        # Now make sure the graph is symmetrical

        for i in range(len(graph)):
          for j in range(len(graph)):
            if i == j:
              continue
            node, other = graph[i], graph[j]
            if node["id"] in other["neighbors"] and other["id"] not in node["neighbors"]:
              node["neighbors"].append(other["id"])

        return graph

    def _add_nodes(self, coords):
      graph = []
      for c in coords:
        x, y = c
        g = {"x":x, "y":y, "id":self._count, "neighbors":set()}
        graph.append(g)
        self._count += 1
      return graph

    def _add_neighbors(self, graph, coords):
        """ Use a simple depth first traversal to find the immediate neighbors of each node

        :param graph: a list of dictionary graph elements
        :param coords: a list of coordinates of interest; the features in the graph image
        """
        for node in graph:
            x, y = node["x"], node["y"]
            for neighbor in self.eight_neighbors((x, y), self._skel):
              u, v = neighbor
              self._traverse(neighbor, node, coords)

    def _traverse(self, start, home, coords):
          seen = set()
          q = deque()
          q.append(start)
          while len(q) > 0:
            curr = q.pop()
            for neighbor in self.eight_neighbors(curr, self._skel):
              if neighbor not in seen and neighbor is not None:
                x, y = neighbor
                seen.add(neighbor)
                if self._skel[(y, x)] > 0:
                  q.append(neighbor)
                if neighbor in coords and neighbor != (home["x"], home["y"]):
                  home["neighbors"].add(neighbor)
                  return


    def eight_neighbors(self, c, map):
      """ Return the indices of the neighboring pixels 

      :param c: an x,y tuple for the point of interest
      :param map: a numpy array representing the image
      :returns a list of x,y tuples
      """
      x, y = c
      width, height = map.shape
      N = (x, max(0, y-1))
      NE = (min(width-1, x+1), max(0, y-1))
      E = (min(width-1, x+1), y)
      SE = (min(width-1, x+1),min(height-1, y+1))
      S = (x, min(height-1, y+1))
      SW = (max(0, x-1),min(height-1, y+1))
      W = (max(0, x-1), y)
      NW = (max(0, x-1),max(0, y-1))
      res = []
      for d in [N, NE, E, SE, S, SW, W, NW]:
        if d != c:
          res.append(d)
        else:
          res.append(None)
      return res

        
def main():
    """Main function"""

    rospy.init_node("graph_server")
    rospy.sleep(2) # wait to connect to rosmaster

    server = Server()
    while not rospy.is_shutdown():
        rospy.spin()
        
if __name__ == "__main__":
    main() 
