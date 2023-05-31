"""
This module contains the functions needed to perform route planning using 
Djikstra's algorithm for the robot in ME/CS/EE 129 Spring '23

Authors: Edward Speer, Garrett Knuf
Date: 5/8/23
"""

from mapping.MapGraph import MapGraph
from queue import PriorityQueue
from constants import heading_map, invert_h_map, UND, BLK, UNB
from mapping.graphics import Visualizer
import pickle


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
        self.q = PriorityQueue()
        self.q.put((0, self.goal))

    def get_goal(self):
        return self.goal.get_location()

    def reset(self, origin):
        """Reinitializes the Djikstra object over the given map to use a 
        different goal node.

        Arguements: origin - the new goal Intersection location    
        """
        for node in self.graph:
            node.reset()
        self.goal = self.graph.get_intersection(origin)
        self.goal.set_cost(0)
        self.q = PriorityQueue()
        self.q.put((0, self.goal))

    def run(self):
        """ Run Djikstra's algorithm on the graph. This will assign a cost and 
        direction to each Intersection in the graph stored in these fields 
        internally in each Intersection object
        """
        while not self.q.empty():
            curr = self.q.get()[1]
            for chile in self.graph.neighbors(curr):
                delta = (curr.get_location()[0] - chile.get_location()[0],
                         curr.get_location()[1] - chile.get_location()[1])
                pot_dir = invert_h_map[delta]
                pot_cost = curr.get_cost() + 1
                if pot_cost < chile.get_cost():
                    if chile.get_cost() != float('inf'):
                        self.q.queue.remove((chile.get_cost(), chile))
                    chile.set_cost(pot_cost)
                    chile.set_dir(pot_dir)
                    self.q.put((chile.get_cost(), chile))

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

    Update: also makes sure intersection returned is not blocked off
    
    Arguments: graph: a MapGraph object to search over 
               curr: the current location of the bot in the graph
    """
    if curr == None:
        return None
    inters = graph.get_intersection(curr)
    # if inters == None:
    #     print("NONE intersection")
    # else:
    #     print(inters.get_blockages())
    nexts = graph.neighbors(inters)
    if len(nexts) != 0:
        for chile in graph.neighbors(inters):
            #print((chile.get_location()))
            if not chile.is_explored():
                # loc2 = inters.get_location()
                # loc1 = chile.get_location()
                # relative_loc = (loc2[0] - loc1[0], loc2[1] - loc1[1])
                # heading = invert_h_map[relative_loc]
                #print(str(loc2) + " " + str(loc1) +)
                #print(chile.get_blockages())
                #print(chile.check_blockage(heading))
                heading = heading_from(inters.get_location(), chile.get_location())
                print("CHILE " + str(chile.get_location()) + " heading" + str(heading))
                print("block found???  " + chile.check_blockage(heading))


                if chile.check_blockage(heading_from(inters.get_location(), chile.get_location())) == UNB:
                    print("next target " + str(chile.location))
                    return chile.location
            nxt = find_unexplored(graph, chile)
            if nxt != None and nxt != curr:
                return nxt
    else:
        #print(str(inters.get_location()) + " HAS NO NIEGHBORSSS")
        # print("Inter (-1, 1) info:")
        # print(graph.get_intersection((-1, 1)).get_streets())
        # print(graph.get_intersection((-1, 1)).get_blockages())

        # print([inter.get_location() for inter in graph.neighbors(graph.get_intersection((-1,1)))])
        # print([inter.get_location() for inter in graph.neighbors(graph.get_intersection((0,1)))])
        # print([inter.get_location() for inter in graph.neighbors(graph.get_intersection((0,2)))])

            
        print("no target found")
        return None
    

def heading_from(loc1, loc2):
    relative_loc = (loc2[0] - loc1[0], loc2[1] - loc1[1])
    return invert_h_map[relative_loc]

def from_pickle(map_num = None):
    """Returns the graph giving the map of a previously explored tape map
    from the stored pickle file of the graph
    """
    if map_num == None:
        map_num = input("Which map are you NormStorming on? (Number): ")
    filename = f'pickles/map{map_num}.pickle'
    print(f'Loading the map from {filename}.')
    toRet = None
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
        if inter.check_connection(i) == UND and inter.check_blockage(i) != BLK:
            return i
    return None
