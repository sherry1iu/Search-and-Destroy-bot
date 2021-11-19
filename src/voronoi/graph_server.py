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
        print(response_graph)
        return resp

    def compute_graph(self):
        """ Compute a topological graph from the provided map """
        while self._map is None:
            pass # block until we have a map

        print("Computing the graph...")
        # Compute the edt on a dilated map so the robot will never hit the wall
        dilated = morphology.binary_dilation(self._map, morphology.square(40))
        d, f = ndimage.distance_transform_edt(dilated, return_indices=True)
        mean = np.mean(d)
        # Now create a thinned skeleton and extract the keypoints from it
        self._skel = morphology.skeletonize(d > mean*THIN)
        corners = corner_peaks(corner_harris(self._skel, k=CORNER_SENS), min_distance=1)
        

        coords = []
        for c in corners:
          coords.append((c[1], c[0])) #x, y
        print("Detected", len(coords), "key points")

        # First do a few traversal to find the neighboring feature nodes
        graph = self._add_nodes(coords)
        for i in range(10): # do this a few times
          self._add_neighbors(graph, coords)

        ids, rev_ids = {}, {} # two dictionaries to help wrangle the nodes
        self._set_ids(graph, ids, rev_ids)

        self._make_graph_symmetrical(graph) # make sure every node's neighbors point to the node

        pruned_graph = self._prune_graph(graph, 100)

        self._add_neighbors(pruned_graph, coords) # this adds all the neighbors back because there's some bug in the pruning that removes all of them
        self._set_ids(pruned_graph, ids, rev_ids)    # convert the raw coordinates added into ids
        
        # remove all the neighbors that aren't actually in the graph 
        for node in pruned_graph:
            to_remove = []
            for neighbor in node["neighbors"]:
                if not self._id_in_graph(neighbor, pruned_graph):
                    to_remove.append(neighbor)
            for item in to_remove:
                node["neighbors"].remove(item)

        for g in pruned_graph: # convert the neighbor sets to lists for JSON
              g["neighbors"] = list(g["neighbors"])
        return pruned_graph 
                

    def _id_in_graph(self, id, graph):
        """
        Search for an id in a graph
        """
        for n in graph:
            if n["id"] == id:
                return True
        return False


    def _set_ids(self, graph, ids, rev_ids):
        # Swap out the positions for the proper ids
        for node in graph:
          ids[(node["x"], node["y"])] = node["id"]
        for node in graph:
          rev_ids[node["id"]] = node

        for node in graph:
          new_neighbors = set() 
          for n in node["neighbors"]:
            new_neighbors.add(ids[n])
          node["neighbors"] = new_neighbors

        # Now make sure the graph is symmetrical
    def _make_graph_symmetrical(self, graph):
        for i in range(len(graph)):
          for j in range(len(graph)):
            if i == j:
              continue
            node, other = graph[i], graph[j]
            if node["id"] in other["neighbors"] and other["id"] not in node["neighbors"]:
              node["neighbors"].add(other["id"])

    def _dist(self, x1, y1, x2, y2):
        """
        Euclidean Distance Helper
        """
        return ((x2-x1)**2 + (y2-y1)**2)**(1/2)

    def _prune_graph(self, graph, thresh=100):
      """
      Removes nodes in the graph that are too close to their neighbors

      :param graph: the graph to prune
      :param thresh: the distance threshold to determine when to prune
      :returns the pruned graph
      """

      to_remove  = set()
      for node in graph:
        if node["id"] in to_remove:
          continue
        for other in graph:
          if node != other:
            if self._dist(other["x"], other["y"], node["x"], node["y"]) < thresh : 
              to_remove.add(other['id'])

      for node in to_remove:
        for neighbor in graph[node]["neighbors"]:
          graph[neighbor]["neighbors"].remove(node)

      new_graph = []
      for i in range(len(graph)):
        if i not in to_remove:
          new_graph.append(graph[i])

      return new_graph


    def _add_nodes(self, coords):
      graph = []
      for c in coords:
        x, y = c
        g = {"x": x, "y": y, "id": self._count, "neighbors": set()}
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
