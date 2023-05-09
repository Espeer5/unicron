"""
This module contains the functions needed to perform route planning using 
Djikstra's algorithm for the robot in ME/CS/EE 129 Spring '23

Authors: Edward Speer, Garrett Knuf
Date: 5/8/23
"""

from mapping.MapGraph import MapGraph
from queue import PriorityQueue
from constants import heading_map, invert_h_map, UND, UNK
from graphics import Visualizer
import pickle

class NodeQueue:
    """ A priority queue of Intersections which is sorted upon 
    insertion and deletion bny cost to maintain the proper execution 
    of Djikstra's algorithm
    
    Inputs: origin - the goal node for the Djikstra Instance
    """
    
    def __init__(self, origin):
        self.Q = PriorityQueue()
        self.Q.put((0, origin))

    def insert(self, node):
        """Insert an Intersection in the correct sorted order by cost
        
        Arguments: node - an Intersection to insert
        """
        self.Q.put((node.get_cost(), node))

    def deq(self):
        """Retrieve, remove, and return the lowest cost intersection"""
        return self.Q.get()

    def empty(self):
        """Check if the NodeQueue is empty"""
        return self.Q.empty()
    
    def size(self):
        """Return the size of the NodeQueue (number of Intersections)"""
        return self.Q.qsize()

    def remove(self, node):
        """Remove a given node from the NodeQueue regardless of cost
        
        Arguments: node - The node to be removed from the Queue
        """
        self.Q.remove(node)


class Djikstra:
    """Contains all the objects needed to run Djikstra's, and 
    the function which computes the shortest path
    
    Inputs: graph - A MapGraph giving the layout of the Intersections/ streets 
            origin - The goal Intersection location of Djikstra's
    """

    def __init__(self, graph, origin):
        self.graph = graph
        self.goal = graph.get_intersection(origin)
        self.goal.set_cost(0)
        self.q = NodeQueue(self.goal)

    def reset(self, origin):
        """Reinitializes the Djikstra object over the given map to use a 
        different goal node.

        Arguements: origin - the new goal Intersection location    
        """
        for node in self.graph:
            node.reset()
        print(origin)
        self.goal = self.graph.get_intersection(origin)
        print(self.goal)
        print(origin == (1,1))
        self.goal.set_cost(0)
        self.q = NodeQueue(self.goal)

    def run(self):
        """ Run Djikstra's algorithm on the graph. This will assign a cost and 
        direction to each Intersewction in the graph stored in these fields 
        internally in each Intersection object
        """
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
        """ Generates a path from the given start point to the goal node of 
        the Djikstra object. Runs Djikstra's, then follows the directions stored 
        in each Intersection until the goal is reached.

        Arguments: start_point - the location where the path begins in the graph

        Returns: path - a list of headings to follow sequentially to reach the 
                        goal.
        """
        path = []
        self.run()
        node = self.graph.get_intersection(start_point)
        while node.get_dir() != None:
            path.append(node.get_dir())
            next_n = (node.get_location()[0] + heading_map[node.get_dir()][0],
                    node.get_location()[1] + heading_map[node.get_dir()][1])
            node = self.graph.get_intersection(next_n)
        return path

def find_unexplored(graph, curr):
    """Uses a DFS to find a nearby intersection with unexplored headings, so 
    that Djikstra may then compute a path to that intersection for exploration
    
    Arguments: graph: a MapGraph object to search over 
               curr: the current location of the bot in the graph
    """
    if curr == None:
        return None
    inters = graph.get_intersection(curr)
    nexts = graph.neighbors(inters)
    if len(nexts) != 0:
        for chile in graph.neighbors(inters):
            if not chile.is_explored():
                return chile.location
            nxt = find_unexplored(graph, chile)
            if nxt != None and nxt != curr:
                return nxt
    else:
        return None
    

def from_pickle():
    """Returns the graph giving the map of a previously explored tape map
    from the stored pickle file of the graph
    """
    map_num = input("Which map are you NormStorming on? (Number): ")
    filename = f'../map{map_num}.pickle'
    print(f'Loading the map from {filename}.')
    try:
        with open(filename, 'rb') as pick:
            toRet = pickle.load(pick)
    except FileNotFoundError:
        print("No such map file found! Aborting")
    return toRet


def init_plan(location, heading):
    """Initializes the variables needed for route planning by a behavior
    
    Arguments: location - the current robot location
               heading - the current robot heading
    """
    graph = MapGraph(location, heading)
    return (graph, Visualizer(graph), Djikstra(graph, (0, 0)))


def unx_dir(inter):
    """Returns a heading which needs to be explored for a given intersection"""
    for i in range(len(inter.get_streets())):
        if inter.check_connection(i) in [UNK, UND]:
            return i
    return None