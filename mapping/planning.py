"""
This module contains the functions needed to perform route planning using 
Djikstra's algorithm for the robot in ME/CS/EE 129 Spring '23

Authors: Edward Speer, Garrett Knuf
Date: 5/8/23
"""

from mapping.MapGraph import MapGraph
from queue import PriorityQueue
import constants as const
from mapping.graphics import Visualizer
import pickle
from math import dist, inf
from constants import heading_map, UNK, UND, DRV, UNB, BLK, NNE


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
                pot_dir = const.invert_h_map[delta]
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
            next_n = (node.get_location()[0] + 
                      const.heading_map[node.get_dir()][0], 
                      node.get_location()[1] + 
                      const.heading_map[node.get_dir()][1])
            node = self.graph.get_intersection(next_n)
        return path


def find_unexplored(graph, curr, seen):
    """Uses a DFS to find a nearby intersection with unexplored headings, so 
    that Djikstra may then compute a path to that intersection for exploration
    Update: also makes sure intersection returned is not blocked off
    
    Arguments: graph: a MapGraph object to search over 
               curr: the current location of the bot in the graph
    """
    if curr == None:
        return None
    inters = graph.get_intersection(curr)
    nexts = graph.neighbors(inters)
    seen.append(curr)
    for chile in nexts:
        if not chile.is_explored():
            heading = heading_from(inters.get_location(), chile.get_location())
            if chile.check_blockage(heading) == const.UNB:
                print("next target " + str(chile.location))
                return chile.location
        if chile.location not in seen:
            nxt = find_unexplored(graph, chile.location, seen)
            if nxt != None and nxt != curr:
                return nxt
    return None
    

def heading_from(loc1, loc2):
    """ Returns the heading that the robot should drive to go from adjacent 
    locations 1 to 2.
    """
    relative_loc = (loc2[0] - loc1[0], loc2[1] - loc1[1])
    return const.invert_h_map[relative_loc]


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


def init_plan(location, heading, prev_loc):
    """Initializes the variables needed for route planning by a behavior
    
    Arguments: location - the current robot location
               heading - the current robot heading
    """
    graph = MapGraph(location, heading, prev_loc)
    return (graph, Visualizer(graph), Djikstra(graph, prev_loc))


def to_head(heading, next_h, graph, location):
    """Computes the direction to turn to achieve a certain heading

    Arguments: heading - the current bot heading
               next_h - the desired next heading of the bot
    """
    if (heading + 4) % 8 == next_h:
        return l_r_unex(graph.get_intersection(location), heading)
    if  next_h in [(heading + i) % 8 for i in range(5)]:
        return "LEFT"
    else: 
        return "RIGHT"
    

def unx_dir(inter):
    """Returns a heading which needs to be explored for a given intersection"""
    for i in range(len(inter.get_streets())):
        if (inter.check_connection(i) == const.UND and 
            inter.check_blockage(i) != const.BLK):
            return i
    return None


def l_r_unex(inter, heading):
    """Determines whether turning left or right is better for exploring"""
    l_list = [inter.check_connection((heading + i) % 8) for i in range(4)]
    r_list = [inter.check_connection((heading + i) % 8) for i in range(4)]
    if const.UND in l_list:
        if const.UND in r_list:
            if r_list.index(const.UND) < l_list.index(const.UND):
                return "RIGHT"
    return "LEFT"

def l_r_nearest_rd(inter, heading):
    """Determines whether turning left or right if better for finding nearest road"""
    l_list = [inter.check_connection((heading + i) % 8) for i in range(1, 5)]
    r_list = [inter.check_connection((heading - i) % 8) for i in range(1, 5)]
    for i in range(4):
        if l_list[i] == const.UND or l_list[i] == const.DRV:
            return "LEFT"
        elif r_list[i] == const.UND or r_list[i] == const.DRV:
            return "RIGHT"
    return None


def l_r_unb(inter, heading):
    """Determines whether left or right is better for finding an unblocked 
    intersection
    """
    l_list = [inter.check_connection((heading + i) % 8) for i in range(4)]
    r_list = [inter.check_connection((heading + i) % 8) for i in range(4)]
    if const.UNB in l_list:
        if const.UNB in r_list:
            if r_list.index(const.UNB) < l_list.index(const.UNB):
                return "RIGHT"
    return "LEFT"

def l_r_s_to_target(inter, heading, target):
    """ Determines whether left, right, or straight is optimal for directed
        explore to a target """

    # Determine if left, right, or straight is closest to target
    location = inter.get_location()
    straight_loc = (location[0] + heading_map[heading][0],
                    location[1] + heading_map[heading][1])
    left_loc = (location[0] + heading_map[(heading + 2) % 8][0],
                location[1] + heading_map[(heading + 2) % 8][1])
    right_loc = (location[0] + heading_map[(heading - 2) % 8][0],
                 location[1] + heading_map[(heading - 2) % 8][1])

    if dist(straight_loc, target) <= dist(left_loc, target) and \
        dist(straight_loc, target) <= dist(right_loc, target) and \
        inter.get_streets()[heading] == UND and inter.get_blockages()[heading] == UNB:
        print("straight")
        return "STRAIGHT"

    go_straight = True
    streets = inter.get_streets()
    blockages = inter.get_blockages()
    for i in range(8):
        if (i % 4) != 0:
            if (streets[(heading + i) % 8] == UNK or streets[(heading + i) % 8] == UND) and blockages[(heading + i) % 8] != BLK:
                go_straight = False
    if go_straight:
        if streets[heading] == UND or streets[heading] == DRV:
            print("straightER")
            return "STRAIGHT"
            
    dir_to_target = "LEFT"
    if dist(target, right_loc) < dist(target, left_loc):
        dir_to_target = "RIGHT"

    # Make sure this direction has unexplored roads
    for i in range(1,4):
        if dir_to_target == "LEFT":
            condition = inter.get_streets()[(heading + i) % 8]
            if (condition == UND or condition == UNK) and inter.get_blockages()[(heading + i) % 8] == UNB:
                return "LEFT"
        else:
            condition = inter.get_streets()[(heading - i) % 8]
            if (condition == UND or condition == UNK) and inter.get_blockages()[(heading - i) % 8] == UNB:
                return "RIGHT"

    # Direction does not have unexplored roads so switch directions
    print("Norman changed his mind about direction to turn")
    if dir_to_target == "LEFT":
        return "RIGHT"
    return "LEFT"

def closest_subtarget(graph, location, heading, target, djik):
    """ Determines the closest subtarget to drive to for directed explore.
        The closest subtarget is an unexplored intersection with a road
        that could potentially connect to an intersection with the closest
        distance to the target """
    unexp_inters = graph.unexp_inters()
    if unexp_inters == []:
        raise ("No unexplored intersections could be found...robot stuck")
    print("Unexplored inters " + str(unexp_inters))
    closest_subtarget = None
    closest_distance = inf
    closest_path_len = inf
    for subtarget in unexp_inters:
        # get heading the robot will face if it travels to the subtarget
        djik.reset(subtarget)
        path = djik.gen_path(location)
        subheading = heading

        if path == []:
            continue

        if len(path) != 0:
            subheading = path[-1]
        # determine distances of adjacent intersections to target
        # make sure that unexplored streets exist in possible directions
        streets = graph.get_intersection(location).get_streets()
        blockages = graph.get_intersection(location).get_blockages()
        distances = []
        for i in range(0, 8):
            if i != 4:
                condition = streets[(subheading + i) % 8]
                blockage = blockages[(subheading + i) % 8]
                if (condition == UND or condition == UNK or condition == DRV) and blockage == UNB:
                    new_loc = (subtarget[0] + heading_map[i][0], subtarget[1] + heading_map[i][1])
                    if i % 2 != 0:
                        distances.append(dist(new_loc, target) + 0.1)
                    else:
                        distances.append(dist(new_loc, target))
        # check if there is a new closest option
        for distance in distances:
            if distance < closest_distance:
                closest_subtarget = subtarget
                closest_distance = distance
                closest_path_len = len(path)
                print(str(subtarget) + " has closest dist " + str(distance))
            elif distance == closest_distance and len(path) < closest_path_len:
                print("Replacing subtarget " + str(closest_subtarget) + " with " + str(subtarget))
                closest_subtarget = subtarget
                closest_distance = distance
                closest_path_len = len(path)
    print("TARG: " + str(closest_subtarget))
    if subtarget == location:
        return None
    return closest_subtarget


    