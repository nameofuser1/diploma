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

        return NODE_TRAPPED

    def _is_q_valid(self, q):
        for i in range(self._ndof):
            if q[i] > self._q_max[i]:
                return False

        return self._is_collision_free(q)

    def _random_sample(self):
        # Vector values x[i] is in [0...1]. Defines direction
        x = np.random.rand(self.ndof)
        x = np.multiply(x, self._q_max)

        return x

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
