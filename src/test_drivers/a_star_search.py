import numpy
import rospy  # module for ROS APIs


class SearchNode:
    """This is just a container for pos, cost, heuristic, f"""

    def __init__(self, pos, cost, heuristic, f, parent):
        self.pos = pos
        self.cost = cost
        self.heuristic = heuristic
        self.f = f
        self.children = []
        self.parent = parent

    def add_child(self, child):
        self.children.append(child)


class AStarSearch:
    """We initialize the grid and the initial and goal states"""

    def __init__(self, initial_pos, goal_pos, grid):
        self.initial_pos = initial_pos
        self.goal_pos = goal_pos
        self.grid = grid
        self.visited_grid = numpy.copy(grid)
        self.priority_queue = []
        self.tree = None
        # this will generally be overridden in perform_search
        self.occupancy_threshold = 0.0
        # this will we be stored so that we can backtrack from the goal to the top of the tree
        self.goal_node = None


        # visited_grid will be filled with 0s and 1s
        for j in range(len(grid)):
            for i in range(len(grid[0])):
                self.visited_grid[j][i] = 0

    @staticmethod
    def calculate_heuristic(pos, goal_pos):
        """
        We use Manhattan distance to calculate the heuristic value from the current position to the goal

        :param pos:           the position we're currently evaluating
        :param goal_pos:      the position we'd like to get to
        :return:              the Manhattan distance between the two points
        """
        return abs(pos["x"] - goal_pos["x"]) + abs(pos["y"] - goal_pos["y"])

    def expand_coordinate(self, pos, parent):
        """expand the coordinate if it passes all checks for that coord"""
        # print(self.grid[pos["y"]][pos["x"]])

        # We check to see if the coordinate has already been visited (i.e added to the grid)
        if self.visited_grid[pos["y"]][pos["x"]] == 0:
            # We check to see if the coordinate is a wall, or exceeds the bounds of our grid
            if (self.grid[pos["y"]][pos["x"]] > self.occupancy_threshold
                    or pos["y"] < 0 or pos["x"] < 0 or pos["y"] >= len(self.grid) or pos["x"] >= len(self.grid[0])
            ):
                # set the visited grid to be visited at that point
                self.visited_grid[pos["y"]][pos["x"]] = 1

            # if it's not a wall, we add it to the priority queue and visit it
            else:
                cost = parent.cost + 1
                heuristic = self.calculate_heuristic(pos, self.goal_pos)
                # instantiate a new node
                child = SearchNode(pos, cost, heuristic, heuristic + cost, parent)
                self.priority_queue.append(child)

                # this line is a little interesting -- it prevents other parents from adding the
                # same child. This is because we assume the earlier parent to have the more optimal
                # trajectory, and we should let that take precedence.
                self.visited_grid[pos["y"]][pos["x"]] = 1
                # add this child to the list of parent children
                parent.add_child(child)

    def expand_node(self, node):
        """expand a node by going in the cardinal directions"""
        # we handle top, bottom, left, right
        self.expand_coordinate({"x": node.pos["x"], "y": node.pos["y"] + 1}, node)
        self.expand_coordinate({"x": node.pos["x"], "y": node.pos["y"] - 1}, node)
        self.expand_coordinate({"x": node.pos["x"] - 1, "y": node.pos["y"]}, node)
        self.expand_coordinate({"x": node.pos["x"] + 1, "y": node.pos["y"]}, node)

    def get_minimum(self):
        """
        Gets the minimum f value for the priority queue, clears that node from the queue

        :return:          the minimum-f node
        """
        min_f = None
        min_f_index = None
        min_f_node = None

        for i in range(len(self.priority_queue)):
            # we try to minimize f
            if (min_f is None or min_f > self.priority_queue[i].f
                    # we break ties by minimizing the heuristic (meaning we tend to move toward the goal)
                    or (min_f == self.priority_queue[i].f and min_f_node.heuristic > self.priority_queue[i].heuristic)
            ):
                min_f_index = i
                min_f_node = self.priority_queue[i]
                min_f = min_f_node.f

        # clears this element from the priority queue
        if min_f_index is None:
            print("An error occurred and min_f_index doesn't exist")

        self.priority_queue.pop(min_f_index)
        return min_f_node

    def perform_backtrack(self):
        """
        This gets the backtrack coordinates by using the parent chain of the tree

        :return:   the raw path from the goal to the start position
        """

        backtrack_chain = []
        current_node = self.goal_node
        backtrack_chain.append(current_node.pos)

        # moves up the chain, which resembles a Linked List in the upward direction of the tree
        while current_node.parent is not None:
            current_node = current_node.parent
            backtrack_chain.append(current_node.pos)

        return backtrack_chain

    def is_path_clear(self, point1, point2):
        """
        Checks to see if the path between the two points is clear, by evaluating all points on
        the path
        :param point1:         the first point
        :param point2:         the second point
        :return:               if the path is clear
        """
        # print("Checking for a clear path between " + str(point1) + " and " + str(point2))

        diff_x = point2["x"] - point1["x"]
        diff_y = point2["y"] - point1["y"]

        is_vertical = False
        slope = None

        if diff_x is 0:
            is_vertical = True
        else:
            slope = float(diff_y) / float(diff_x)

        if slope is not None and slope > 1 or slope < -1:
            is_vertical = True

        # handle the case if point2["x"] < point1["x"]
        did_flip = False

        if diff_x < 0:
            did_flip = True
            diff_x = -diff_x

        if is_vertical:
            if diff_y < 0:
                did_flip = True
                diff_y = -diff_y

            for i in range(diff_y):
                offset = 0
                if slope is not None:
                    offset = i / slope

                if not did_flip:
                    y_eval = point1["y"] + i
                    x_eval = point1["x"] + offset
                else:
                    y_eval = point2["y"] + i
                    x_eval = point2["x"] + offset

                if self.grid[int(y_eval)][int(x_eval)] > self.occupancy_threshold:
                    return False
        else:
            for i in range(diff_x):
                # we evaluate all along the line
                if not did_flip:
                    y_eval = point1["y"] + slope * i
                    x_eval = point1["x"] + i
                else:
                    y_eval = point2["y"] + slope * i
                    x_eval = point2["x"] + i

                # if we encounter a wall, we'll just return
                if self.grid[int(y_eval)][x_eval] > self.occupancy_threshold:
                    return False

        # otherwise we have a clear path
        return True

    def minimize_path(self, raw_path):
        """
        Minimizes the number of rotations by checking nearby nodes to see if there
        is a clear path between them.
        NOTE: raw_path will be goal-position first; this will return goal-position last.

        :param raw_path:       the path before optimization
        :return:               the optimized path
        """
        new_path = []
        # add the end point
        new_path.append(raw_path[-1])

        print ("Minimizing the path")

        i_max = len(raw_path)
        while i_max > 0 and not rospy.is_shutdown():
            # starting from 0, going to i_max, we keep incrementing until the lower point is accessible
            # from the point at i_max
            for i in range(i_max - 1):
                # if the path is clear, we add that
                if self.is_path_clear(raw_path[i], raw_path[i_max - 1]):
                    # decrease i_max, add the node
                    i_max = i
                    # print ("New i_max is: " + str(i_max))
                    new_path.append(raw_path[i])
                    break

        return new_path

    def perform_search(self, occupancy_threshold):
        """
        Conducts A* search

        :param occupancy_threshold:          the decimal threshold a cell has to be greater than to be "occupied"
        :return:                             the optimized set of points the robot will route through
        """
        # use our preferred occupancy threshold
        self.occupancy_threshold = occupancy_threshold
        # clear goal_node
        self.goal_node = None

        print("Initial pos: " + str(self.initial_pos))
        print("Goal pos: " + str(self.goal_pos))

        initial_heuristic = self.calculate_heuristic(self.initial_pos, self.goal_pos)
        self.tree = SearchNode(self.initial_pos, 0, initial_heuristic, initial_heuristic, None)
        next_node = self.tree

        has_completed = False
        while not has_completed and not rospy.is_shutdown():
            # if we're at the goal, complete and break
            if self.goal_pos["x"] == next_node.pos["x"] and self.goal_pos["y"] == next_node.pos["y"]:
                has_completed = True
                # we set the goal_node for future retrieval
                self.goal_node = next_node

            else:
                self.expand_node(next_node)
                next_node = self.get_minimum()
                # print("Next node: " + str(next_node.pos) + ", queue length: " + str(len(self.priority_queue)))

        # gets the raw path based on A* search
        raw_path = self.perform_backtrack()

        print ("Search complete")

        # cleans up the path by checking nearby nodes to see if there is a clear path between them
        return self.minimize_path(raw_path)
