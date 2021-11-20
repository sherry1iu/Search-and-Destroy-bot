import numpy
import rospy  # module for ROS APIs


class SearchNode:
    """This is just a container for pos, cost, heuristic, f"""

    def __init__(self, pos, parent):
        self.pos = pos
        self.children = []
        self.parent = parent

    def add_child(self, child):
        self.children.append(child)


class BFS:
    """We initialize the grid and the initial and goal states"""

    def __init__(self, initial_pos, goal_pos_possibilities, grid):
        self.initial_pos = initial_pos
        self.goal_pos_possibilities = goal_pos_possibilities
        self.grid = grid
        self.visited_grid = numpy.copy(grid)
        self.queue = []
        self.tree = None
        # this will generally be overridden in perform_search
        self.occupancy_threshold = 0.0
        # this will we be stored so that we can backtrack from the goal to the top of the tree
        self.goal_node = None

        # visited_grid will be filled with 0s and 1s
        for j in range(len(grid)):
            for i in range(len(grid[0])):
                self.visited_grid[j][i] = 0

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
                # instantiate a new node
                child = SearchNode(pos, parent)
                self.queue.append(child)

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
        # add the end point
        new_path = [raw_path[-1]]

        print ("Minimizing the path")

        i_max = len(raw_path)
        if i_max is 1:
            new_path.append(raw_path[0])
        while i_max > 1 and not rospy.is_shutdown():
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
        Conducts BFS search

        :param occupancy_threshold:          the decimal threshold a cell has to be greater than to be "occupied"
        :return:                             the optimized set of points the robot will route through
        """
        # use our preferred occupancy threshold
        self.occupancy_threshold = occupancy_threshold
        # clear goal_node
        self.goal_node = None

        print("Initial pos: " + str(self.initial_pos))
        print("Goal pos: " + str(self.goal_pos_possibilities))

        self.tree = SearchNode(self.initial_pos, None)
        next_node = self.tree
        self.queue = [next_node]

        has_completed = False
        has_failed = False
        while not has_completed and not rospy.is_shutdown():
            # if we're at the goal, complete and break
            did_find_goal = False

            # There are as many possible goals as nodes on the graph; we check all of them.
            for goal_pos in self.goal_pos_possibilities:
                if goal_pos["x"] == next_node.pos["x"] and goal_pos["y"] == next_node.pos["y"]:
                    has_completed = True
                    # we set the goal_node for future retrieval
                    self.goal_node = next_node
                    did_find_goal = True

            print(str(len(self.queue)))

            if len(self.queue) is 0:
                has_completed = True
                has_failed = True
                print ("Failed: returning an empty list...")

            elif not did_find_goal:
                self.expand_node(next_node)
                next_node = self.queue.pop(0)
                # print("Next node: " + str(next_node.pos) + ", queue length: " + str(len(self.queue)))

        if has_failed:
            return []

        # gets the raw path based on A* search
        raw_path = self.perform_backtrack()

        print ("Search complete")

        # cleans up the path by checking nearby nodes to see if there is a clear path between them
        return self.minimize_path(raw_path)
