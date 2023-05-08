"""
This module contains the functions needed to perform route planning using Djikstra's 
algorithm for the robot in ME/CS/EE 129 Spring '23

Authors: Edward Speer, Garrett Knuf
Date: 5/8/23
"""

from mapping.MapGraph import MapGraph, Intersection
from queue import PriorityQueue

class NodeQueue:
    """ A priority queue of SeachNodes which is sorted upon 
    insertion and deletion to maintain the proper execution order 
    of Djikstra's algorithm"""
    
    def __init__(self, origin):
        self.Q = PriorityQueue()
        self.Q.put((0, origin))

    def insert(self, node):
        self.Q.put((node.get_cost(), node))

    def deq(self):
        return self.Q.get()

    def empty(self):
        return self.Q.empty()
    
    def size(self):
        return self.Q.qsize()

    def remove(self, node):
        self.Q.remove(node)


class Djikstra:
    """Contains all the objects needed to run Djikstra's, and 
    the function which computes the shortest path"""

    def __init__(self, graph, origin):
        self.graph = graph
        self.goal = graph.get_intersection(origin)
        self.goal.set_cost(0)
        self.q = NodeQueue(self.goal)

    def reset(self, origin):
        for node in self.graph:
            node.reset()
        self.goal = self.graph.get_intersection(origin)
        self.goal.set_cost(0)
        self.q = NodeQueue(self.goal)

    def run(self):
        invert_h_map= {(0, 1): 0, (1, 1): 1, (1, 0): 2, (1, -1):3,
                       (0, -1):4, (-1, -1): 5, (-1, 0): 6, (-1, 1): 7}
        while not self.q.empty():
            curr = self.q.deq()[1]
            for chile in self.graph.neighbors(curr):
                delta = (curr.get_location()[0] - chile.get_location()[0],
                         curr.get_location()[1] - chile.get_location()[1])
                pot_dir = invert_h_map[delta]
                pot_cost = curr.get_cost() + 1
                if pot_cost < chile.get_cost():
                    if chile.get_cost() != float('inf'):
                        self.q.remove(chile)
                    chile.set_cost(pot_cost)
                    chile.set_dir(pot_dir)
                    self.q.insert(chile)

    def gen_path(self, start_point):
        heading_map = {0:(0, 1), 1:(1, 1), 2:(1, 0), 3:(1, -1),
                4:(0, -1), 5:(-1, -1), 6:(-1, 0), 7:(-1, 1)}
        path = []
        self.run()
        node = self.graph.get_intersection(start_point)
        while node.get_dir() != None:
            path.append(node.get_dir())
            next_n = (node.get_location()[0] + heading_map[node.get_dir()][0],
                    node.get_location()[1] + heading_map[node.get_dir()][1])
            node = self.graph.get_intersection(next_n)
        return path


