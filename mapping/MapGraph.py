"""
This module contains the objects which make up the map representation
stored and tracked by the robot as it navigates mazes in ME/CS/EE 129.

Authors: Edward Speer, Garrett Knuf
Date: 4/30/23
"""

import os
from constants import CONDITIONS, UNK, UND, NNE, DRV


class Intersection:
    """ An intersection object maps each heading from an intersection
    to a label which describes the bot's knowledge of whether a road
    is present in any of the 8 possible bot headings

    Inputs: curr_heading - the heading which the bot first approached
    the intersection from
    """

    def __init__(self, location, heading=None):
        self.location = location
        self.cost = float('inf')
        self.direction = None
        self.streets = dict.fromkeys(range(8), UNK)
        if heading != None:
            self.streets[(heading + 4) % 8] = DRV

    def __lt__(self, other):
        """Comparison method less than used for comparing two Intersection so 
        that they may be ordered in a PriorityQueue used to implement Djikstra's
        """
        return self.cost < other.cost
            
    def set_connection(self, heading, status):
        """ Set the label of a certain direction in the intersection 
            object.

            Arguments: heading - the direction to set a label for
                       status - the label to set the heading to
        """
        if status not in CONDITIONS:
            raise Exception("Intersection.set_connection: Invalid status")
        else:
            self.streets[heading] = status

    def get_streets(self):
        """Return the streets list form an Intersection"""
        return self.streets

    def reset(self):
        """Reset the cost of an Intersection to infinity and the optimal leaving 
        direction to None. For initializing Djikstra's
        """
        self.cost = float('inf')
        self.direction = None

    def set_cost(self, cost):
        """Sets the cost associated with the Intersection to cost"""
        self.cost = cost

    def get_cost(self):
        """Returns the cost associated with the Intersection"""
        return self.cost

    def get_dir(self):
        """Returns the optimal leaving direction from the Intersection"""
        return self.direction

    def set_dir(self, direc):
        """Sets the optimal leaving direction of an Intersection to direc"""
        self.direction = direc

    def get_location(self):
        """Returns the location tuple associated with the Intersection"""
        return self.location

    def print(self):
        """Prints relevant information about an intersection"""
        print("Loc: " + self.location + " Cost: " + self.cost +
                " Dir: " + self.direction)

    def check_connection(self, heading):
        """
        returns the label associated with a certain heading from the
        intersection
        """
        return self.streets[heading]

    def is_explored(self):
        """
        Returns a boolean indicating whether an intersection is fully
        explored, meaning it has no unknown or undriven streets
        """
        labels = self.streets.values()
        return UNK not in labels and UND not in labels


class MapGraph:
    """
    This object is a graph representing the set of all intersections found by 
    the bot (vertices) and the streets connecting each of those intersection (edges)
    """

    def invert_heading(self, heading):
        """ A simple utility which returns the opposite heading (180 degrees) given
        a heading
        """
        return (heading + 4) % 8

    def __init__(self, location, heading):
        origin = Intersection((0, 0))
        point = Intersection(location, heading)
        self.graph = {point:[origin], origin:[point]}

    def get_graph(self):
        """
        Returns the dictionary representing the graph of the map
        """
        return self.graph

    def driven_connection(self, prev_location, location, heading):
        """
        Sets the streets for each intersection when a connection 
        between 2 intersections has been driven

        Arguments: prev_location: the last visited intersection location
                   location: the location of the current intersection
                   heading: the current heading of the bot
        """
        prev_inters = self.get_intersection(prev_location)
        prev_inters.set_connection(heading, DRV)
        inters = self.get_intersection(location)
        if inters == None:
            inters = Intersection(location, heading)
            self.graph[inters] = []
        inters.set_connection(self.invert_heading(heading), DRV)
        if prev_inters not in self.graph[inters]:
            self.graph[inters].append(prev_inters)
        if inters not in self.graph[prev_inters]:
            self.graph[prev_inters].append(inters)

    def no_connection(self, location, heading):
        """
        Marks that a street doesn't exist in a certain direction for an 
        intersection in the graph

        Arguments: location: the current bot location
                   heading: the heading for which no street exists
        """
        self.get_intersection(location).set_connection(heading, NNE)

    def markoff(self, location, angle, start_head, direction):
        """
        Updates the graph after the robot has completed a turn, marking those 
        headings without streets as "NONE" and those with newly found streets
        as undriven

        Arguments: location: the current bot coordinates
                   angle: the angle covered by the bot turn
                   start_head: the heading the bot was facing before the turn
                   direction: Left/right for the turn (casing/letters enforced
                   in input code)
        """
        dir_map = {'L': 1, 'R':-1}
        start_head += dir_map[direction]
        inters = self.get_intersection(location)
        for i in range(round(angle / 45) - 1):
            heading = (start_head + (dir_map[direction] * i)) % 8
            if inters.check_connection(heading) not in [UNK, NNE]:
                raise Exception("Expected intersection is missing, aborting")
            self.no_connection(location, heading)
        heading = (start_head + dir_map[direction] * (round(angle / 45) - 1)) % 8
        if inters.check_connection(heading) == NNE:
            raise Exception("Nonexisting road found. Aborting")
        if inters.check_connection(heading) != DRV:
            inters.set_connection(heading, UND)


    def contains(self, location):
        """
        Returns true if an intersection location is in the map, false otherwise
        """
        for intersection in self.graph:
            if intersection.get_location() == location:
                return True
        return False

    def is_complete(self):
        """
        Returns true if all intersections in the map are fully explored, 
        false otherwise
        """
        for intersection in self.graph:
            if not intersection.is_explored():
                return False
        return len(self.graph) != 0 

    def get_intersection(self, location):
        """
        Given a location, extracts the corresponding intersection object 
        from the graph
        """
        for intersection in self.graph:
            if location == intersection.get_location():
                return intersection
        return None

    def neighbors(self, inters):
        """
        Returns the list of adjacent Intersections for a given Intersection 
        inters in the graph
        """
        if inters == None:
            return []
        return self.graph[inters]

    def __iter__(self):
        """
        Define an iterator over a MapGraph as an iterator over the dictionary
        mapping vertices to other vertices in the graph
        """
        return iter(self.graph)
    
