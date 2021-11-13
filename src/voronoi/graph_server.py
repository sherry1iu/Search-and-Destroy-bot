#!/usr/env python
import json
import rospy
import numpy as np
from scipy import ndimage
from skimage import morphology
from skimage.feature import corner_harris, corner_peaks
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
        while self._map is None:
            pass # block until we have a map

        d, f = ndimage.distance_transform_edt(self._map, return_indices=True)
        mean = np.mean(d)
        self._skel = morphology.skeletonize(d > mean*THIN)
        corners = corner_peaks(corner_harris(self._skel, k=CORNER_SENS), min_distance=1)

        coords = []
        for c in corners:
          coords.append((c[1], c[0])) #x, y

        # First do DFS to find the neighboring feature nodes

        graph = []
        self.dfs(None, coords[0], graph, set(), coords)

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

    def dfs(self, last, current, graph, seen, coords):
      l = last
      seen.add(current)
      if current in coords:
        x, y = current
        g = {"x":x, "y":y, "id":self._count, "neighbors":[last]}
        if last is None:
          g["neighbors"] = []
        graph.append(g)
        self._count += 1
        l = current
      for n in self.eight_neighbors(current, self._skel):
        if n is None: # wtf?
            return
        x, y = n
        if self._skel[(y, x)] > 0 and (x, y) not in seen:
          self.dfs(l, n, graph, seen, coords)

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
