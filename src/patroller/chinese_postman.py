from src.patroller.dijkstra import DijkstraSearch


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

    @staticmethod
    def navigate_between_nodes(start_node, end_node, edge_dictionary):
        """Finds the optimal path from the start to end node for the robot using Dijkstra's algo"""
        dijkstra = DijkstraSearch(initial_id=start_node, goal_id=end_node, edge_dictionary=edge_dictionary)
        path = dijkstra.perform_search()
        print(path)
        return path

    @staticmethod
    def get_cost_of_path(path, edge_dictionary):
        """Gets the cost of this path"""
        cost = 0
        if len(path) > 1:
            for i in range(len(path) - 1):
                # sums the cost between adjacent
                cost += edge_dictionary[path[i]][path[i+1]]

        return cost

    @staticmethod
    def hierholzers_method(starting_node, edge_dictionary):
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
                # remove from the other side as well
                if curr_v in remaining_edges[next_v]:
                    remaining_edges[next_v].remove(curr_v)
                    edge_count[next_v] -= 1

                # Move to next vertex
                curr_v = next_v

            # back-track to find remaining circuit
            else:
                circuit.append(curr_v)

                # Back-tracking
                curr_v = curr_path[-1]
                curr_path.pop()

        # return the reversed circuit
        circuit.reverse()
        return circuit

    def find_cheapest_pairs(self, cost_dictionary, pairs, used_nodes, cumulative_cost):
        """Finds the cheapest world where all the nodes are in pairs; calls itself recursively"""
        min_cost = None
        min_cost_nodes = None
        did_recurse = False

        for _id1 in cost_dictionary:
            if _id1 not in used_nodes:
                for _id2 in cost_dictionary[_id1]:
                    if _id2 not in used_nodes and not _id1 == _id2:
                        temp_pairs = pairs[:]
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

    def create_fake_nodes(self, node_pairs, edge_dictionary_copy, edge_dictionary, edge_to_cost_dictionary):
        """Creates fake, otherwise unconnected links between each node in node_pairs"""
        # modifies the edge_dictionary to add fake nodes in the middle of each odd pair. This allows us to use those
        # connections to traverse the graph twice for the linked pairs.
        for i in range(len(node_pairs)):
            node_path = self.navigate_between_nodes(node_pairs[i][0], node_pairs[i][1], edge_dictionary)

            # if the len is 2, then we add a node in the middle of the connection that we will remove later
            if len(node_path) == 2:
                fake_node_name = "fake_node_remove_" + str(i)
                edge_dictionary_copy[fake_node_name] = {}
                cost = edge_to_cost_dictionary[node_pairs[i][0]][node_pairs[i][1]]
                # puts 1/2 the cost into every edge of the graph
                edge_dictionary_copy[fake_node_name][node_pairs[i][0]] = cost/2
                edge_dictionary_copy[fake_node_name][node_pairs[i][1]] = cost/2
                edge_dictionary_copy[node_pairs[i][0]][fake_node_name] = cost/2
                edge_dictionary_copy[node_pairs[i][1]][fake_node_name] = cost/2

            # otherwise, the len > 2 and we will copy the middle nodes to create fake nodes. After pathing, we
            # will return these to be the original value of the nodes
            else:
                for j in range(len(node_path) - 1):
                    low_node = node_path[j]
                    high_node = node_path[j+1]
                    cost = edge_dictionary[low_node][high_node]

                    # create new nodes in the middle; use them to patch together an alternative path
                    if j > 0:
                        fake_node_name = "fake_node_" + str(node_path[j])
                        if fake_node_name not in edge_dictionary_copy:
                            edge_dictionary_copy[fake_node_name] = {}
                        low_node = fake_node_name

                    if j+1 < len(node_path) - 1:
                        fake_node_name = "fake_node_" + str(node_path[j+1])
                        if fake_node_name not in edge_dictionary_copy:
                            edge_dictionary_copy[fake_node_name] = {}
                        high_node = fake_node_name

                    edge_dictionary_copy[low_node][high_node] = cost
                    edge_dictionary_copy[high_node][low_node] = cost

    def find_cpp_loop(self, starting_node, node_dictionary, edge_dictionary):
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

        edge_dict_copy = edge_dictionary.copy()
        # creates fake nodes so that we can find a cycle
        self.create_fake_nodes(
            node_pairs=cheapest_pairs,
            edge_dictionary=edge_dictionary,
            edge_dictionary_copy=edge_dict_copy,
            edge_to_cost_dictionary=edge_to_cost_dictionary
        )
        raw_path = self.hierholzers_method(starting_node, edge_dictionary=edge_dict_copy)

        cleaned_path = []
        for _id in raw_path:
            # remove the removable nodes in the path
            if str(_id).find("fake_node_remove_") == -1:
                # exchange the other nodes for their ream counterparts
                if str(_id).find("fake_node_") == -1:
                    cleaned_path.append(_id)
                else:
                    # remove the string parts and convert to an int
                    cleaned_path.append(int(str.replace(_id, "fake_node_", "")))

        # store an instance variable that we may recover later
        self.closed_cpp_loop = cleaned_path
        print(cleaned_path)
        return cleaned_path
