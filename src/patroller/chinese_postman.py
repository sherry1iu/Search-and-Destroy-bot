class ChinesePostmanProblem:
    """
    I use an adaptation of the pseudocode and R code found here:
    https://freakonometrics.hypotheses.org/53694

    I need to do something to make sure I get a closed loop --> perhaps I solve the CPP, navigate the robot to
    the starting node, and then include a reset path from the ending node to starting node?
    """
    def __init__(self):
        # a closed cycle where the start and end nodes are neighbors
        self.closed_cpp_loop = []

    def navigate_between_nodes(self, start_node, end_node, edge_dictionary):
        """Finds the optimal path from the start to end node for the robot using Dijkstra's algo"""

        return []

    def get_cost_of_path(self, path, edge_dictionary):
        """Gets the cost of this path"""
        cost = 0
        if len(path) > 1:
            for i in range(len(path) - 1):
                # sums the cost between adjacent
                cost += edge_dictionary[path[i]][path[i+1]]

        return cost

    def hierholzers_method(self, starting_node, edge_dictionary):
        """
        Given a graph of nodes of entirely even degree, finds a Eulerian cycle in constant time

        Method adapted from: https://www.geeksforgeeks.org/hierholzers-algorithm-directed-graph/
        """
        edge_count = {}
        remaining_edges = {}
        # Maintain a stack to keep vertices
        curr_path = []
        # vector to store final circuit
        circuit = []
        # start from any vertex
        curr_path.append(starting_node)
        curr_v = starting_node # Current vertex

        for _id in edge_dictionary:
            # find the count of edges to keep track of unused edges
            edge_count[_id] = len(edge_dictionary[_id])
            remaining_edges[_id] = edge_dictionary[_id].keys()

        while len(curr_path):
            # If there's remaining edge
            if edge_count[curr_v]:
                # Push the vertex
                curr_path.append(curr_v)
                # Find the next vertex using an edge
                next_v = remaining_edges[curr_v].pop()
                edge_count[curr_v] -= 1

                # Move to next vertex
                curr_v = next_v

            # back-track to find remaining circuit
            else:
                circuit.append(curr_v)

                # Back-tracking
                curr_v = curr_path[-1]
                curr_path.pop()

        # return the reversed circuit
        return circuit.reverse()

    def find_cheapest_pairs(self, cost_dictionary, pairs, used_nodes, cumulative_cost):
        """Finds the cheapest world where all the nodes are in pairs; calls itself recursively"""
        min_cost = None
        min_cost_nodes = None
        did_recurse = False

        for _id1 in cost_dictionary:
            if not used_nodes[_id1]:
                for _id2 in cost_dictionary:
                    if not used_nodes[_id2] and not _id1 == _id2:
                        temp_pairs = pairs.copy()
                        temp_pairs.append((_id1, _id2))
                        temp_used_nodes = used_nodes.copy()
                        temp_used_nodes[_id1] = True
                        temp_used_nodes[_id2] = True
                        min_cost_prop, min_cost_prop_nodes = self.find_cheapest_pairs(
                            cost_dictionary=cost_dictionary,
                            pairs=temp_pairs,
                            used_nodes=temp_used_nodes,
                            cumulative_cost=cumulative_cost+cost_dictionary[_id1][_id2]
                        )

                        if min_cost is None or min_cost_prop < min_cost:
                            min_cost = min_cost_prop
                            min_cost_nodes = min_cost_prop_nodes
                            did_recurse = True
        # if we're at the lowest layer, return the pairs and cost
        if not did_recurse:
            return cumulative_cost, pairs

        # otherwise, returns the minimum cost and the nodes associated with that minimum cost
        return min_cost, min_cost_nodes

    def find_cpp_loop(self, node_dictionary, edge_dictionary):
        """Finds a cpp loop -- this may not be ideal because of the connection we have to make at the end"""
        if len(edge_dictionary) == 0:
            return

        # a list of the ids of all the odd-degree vertices
        odd_degree_vertices = []
        # double nested dictionary of node1_id -> node2_id -> path_cost
        edge_to_cost_dictionary = {}

        for _id in node_dictionary:
            # if the vertex is of odd degree, add it to that array
            if len(edge_dictionary[_id]) % 2 != 0:
                odd_degree_vertices.append(_id)

        # this step takes O(n^2)*BFS_COST time, where n is the number of odd vertices
        for i in range(len(odd_degree_vertices) - 1):
            edge_to_cost_dictionary[odd_degree_vertices[i]] = {}
            for j in range(len(odd_degree_vertices) - i - 1):
                j_inc = j + i + 1

                edge_to_cost_dictionary[odd_degree_vertices[i]][odd_degree_vertices[j_inc]] = self.get_cost_of_path(
                    self.navigate_between_nodes(
                        start_node=odd_degree_vertices[i],
                        end_node=odd_degree_vertices[j_inc],
                        edge_dictionary=edge_dictionary,
                    ),
                    edge_dictionary
                )

        # finds the cheapest world where all given nodes are in pairs
        cheapest_pairs = self.find_cheapest_pairs(edge_to_cost_dictionary, [], {}, 0)[1]

        # modifies the edge_dictionary to add fake nodes in the middle of each odd pair. This allows us to use those
        # connections to traverse the graph twice for the linked pairs.

