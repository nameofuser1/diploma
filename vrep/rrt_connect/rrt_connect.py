import numpy as np


NODE_REACHED = 0
NODE_ADVANCED = 1
NODE_TRAPPED = 2


def dummy_collision_check(q0):
    return True


class RRT(object):

    class Node(object):

        def __init__(self, data):
            self._data = data
            self._children = []

        @property
        def data(self):
            return self._data

        @data.setter
        def data(self, data):
            self._data = data

        @property
        def children(self):
            return self._children

    def __init__(self, q_init, q_max, ndof, epsilon=0.3,
                 collision_free_fcn=dummy_collision_check,
                 eps=0.001):
        self._tree = RRT.Node(np.asarray(q_init))
        self._q_max = np.asarray(q_max)
        self._ndof = ndof
        self._epsilon = epsilon
        self._is_collision_free = collision_free_fcn
        self._q_new = None
        self._eps = 0.001

    def extend(self, q):
        new_node_q = None

        if self._is_q_valid(q):
            # Find nearest node in tree
            q_diff, nn = self._find_nearest(q)

            # Normalize vector and make it epsilon length
            q_norm = (q / np.linalg.norm(q)) * min(self.epsilon, q_diff)
            new_node_q = np.add(nn.data, q_norm)

            # Append new node to a tree
            nn.children.append(RRT.Node(new_node_q))

            if np.linalg.norm(np.subtract(new_node_q, q)) <= self._eps:
                return NODE_REACHED
            else:
                return NODE_ADVANCED

        return NODE_TRAPPED, new_node_q

    def build_path_to_node(self, q):
        pass

    def _is_q_valid(self, q):
        for i in range(self._ndof):
            if q[i] > self._q_max[i]:
                return False

        return self._is_collision_free(q)

    def _find_nearest(self, q):
        root = self._tree
        self._q_new = q
        nearest_val, nearest_node = self._walk_node(root)

        return nearest_val, nearest_node

    def _walk_node(self, node):
        nearest_node = node
        nearest_val = np.linalg.norm(np.subtract(self._q_new, node.data))

        for child in node.children:
            new_val, new_node = self._walk_node(child)

            if(new_val < nearest_val):
                nearest_node = new_node
                nearest_val = new_val

        return (nearest_val, nearest_node)


class RRTPlanner(object):

    def __init__(self, ndof, q_max, epsilon=0.3, max_iterations=5000,
                 collision_free_fcn=dummy_collision_check, eps=0.001):
        self._ndof = ndof
        self._q_max = q_max
        self._epsilon = epsilon
        self._collision_fcn = collision_free_fcn
        self._eps = eps
        self._max_iteration = max_iterations

    def plan(self, q_init, q_goal):
        rrt1 = RRT(q_init, self._q_max, self._ndof, self._epsilon,
                   self._collision_free_fcn, self._eps)

        rrt2 = RRT(q_goal, self._q_max, self._ndof, self._epsilon,
                   self._collision_fcn, self._eps)

        for i in range(self._max_iteration):
            q_rand = self._random_sample()

            node_state, q_new = rrt1.extend(q_rand)

            if node_state != NODE_TRAPPED:
                if self._connect(rrt2, q_new) == NODE_REACHED:
                    # Return path
                    return self._build_path()

            # Swap trees
            s_rrt = rrt1
            rrt1 = rrt2
            rrt2 = s_rrt

        return None

    def _build_path(self, rrt1, rrt2, q):
        path1 = rrt1.build_path_to_node(q)
        path2 = rrt2.build_path_to_node(q)

        return self.__merge_paths(path1, path2)

    def _connect(self, rrt, q):
        node_state = NODE_ADVANCED

        while node_state == NODE_ADVANCED:
            node_state = rrt.extend(q)

        return node_state

    def _random_sample(self):
        # Vector values x[i] is in [0...1]. Defines direction
        x = np.random.rand(self.ndof)
        x = np.multiply(x, self._q_max)

        return x

    def __merge_paths(self, path1, path2):
        path = list(path1)

        # If path contains only root point
        if len(path2) > 1:
            path.extend(path2[1:])

        return path
