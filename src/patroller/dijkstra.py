import numpy
import rospy  # module for ROS APIs


class SearchNode:
    """This is just a container for node_id, cost"""

    def __init__(self, node_id, cost, parent):
        self.node_id = node_id
        self.cost = cost
        self.children = []
        self.parent = parent

    def add_child(self, child):
        self.children.append(child)


class DijkstraSearch:
    """Methods for using Dijkstra's Algo on a graph"""

    def __init__(self, initial_id, goal_id, edge_dictionary):
        self.initial_id = initial_id
        self.goal_id = goal_id
        self.edge_dictionary = edge_dictionary
        self.expanded_nodes = {}

        self.priority_queue = []
        self.tree = None
        # this will we be stored so that we can backtrack from the goal to the top of the tree
        self.goal_node = None

    def add_child_node(self, node_id, parent):
        """expand the child node, keeping track of cost w.r.t origin"""
        # We check to see if the node has already been visited (i.e added to the tree)
        if node_id not in self.expanded_nodes or self.expanded_nodes[node_id]:
            self.expanded_nodes[node_id] = True

            cost = parent.cost + self.edge_dictionary[parent.node_id][node_id]
            # instantiate a new node
            child = SearchNode(node_id, cost, parent)
            self.priority_queue.append(child)
            parent.add_child(child)

    def expand_node(self, node):
        """Expand a node"""
        for node_id in self.edge_dictionary[node.node_id]:
            if node_id not in self.expanded_nodes:
                self.add_child_node(node_id, node)

    def get_minimum(self):
        """
        Gets the minimum cost value for the priority queue, clears that node from the queue

        :return:          the minimum-cost node
        """
        min_cost = None
        min_cost_index = None
        min_cost_node = None

        for i in range(len(self.priority_queue)):
            # we try to minimize cost
            if min_cost is None or min_cost > self.priority_queue[i].cost:
                min_cost_index = i
                min_cost_node = self.priority_queue[i]
                min_cost = min_cost_node.cost

        # clears this element from the priority queue
        if min_cost_index is None:
            print("An error occurred and min_cost_index doesn't exist")

        self.priority_queue.pop(min_cost_index)
        return min_cost_node

    def perform_backtrack(self):
        """
        This gets the backtrack coordinates by using the parent chain of the tree

        :return:   the raw path from the goal to the start to goal
        """

        backtrack_chain = []
        current_node = self.goal_node
        backtrack_chain.append(current_node.node_id)

        # moves up the chain, which resembles a Linked List in the upward direction of the tree
        while current_node.parent is not None:
            current_node = current_node.parent
            backtrack_chain.append(current_node.node_id)

        return backtrack_chain

    def perform_search(self):
        """
        Conducts Dijkstra search

        :return:                             the set of nodes the robot will route through
        """
        # clear goal_node
        self.goal_node = None

        self.tree = SearchNode(self.initial_id, 0, None)
        next_node = self.tree

        has_completed = False
        while not has_completed and not rospy.is_shutdown():
            # if we're at the goal, complete and break
            if self.goal_id == next_node.node_id:
                has_completed = True
                # we set the goal_node for future retrieval
                self.goal_node = next_node

            else:
                self.expand_node(next_node)
                next_node = self.get_minimum()
                # print("Next node: " + str(next_node.node_id) + ", queue length: " + str(len(self.priority_queue)))

        raw_path = self.perform_backtrack()

        # reverses the backtrack
        raw_path.reverse()
        return raw_path
