"""
This module contains the objects which make up the map representation
stored and tracked by the robot as it navigates mazes in ME/CS/EE 129.

Authors: Edward Speer, Garrett Knuf
Date: 4/30/23
"""

import os


class Intersection:
    """ An intersection object maps each heading from an intersection
    to a label which describes the bot's knowledge of whether a road
    is present in any of the 8 possible bot headings

    Inputs: curr_heading - the heading which the bot first approached
    the intersection from
    """

    # Discrete set of labels available for each heading at an intersection
    CONDITIONS = ["UNKNOWN", "UNDRIVEN", "NONE", "DRIVEN"]

    def __init__(self, location, heading=None):
        self.location = location
        self.streets = dict.fromkeys(range(8), "UNKNOWN")
        if heading != None:
            self.streets[(heading + 4) % 8] = "DRIVEN"
            
    def set_connection(self, heading, status):
        """ Set the label of a certain direction in the intersection 
            object.

            Arguments: heading - the direction to set a label for
                       status - the label to set the heading to
        """
        if status not in self.CONDITIONS:
            raise Exception("Intersection.set_connection: Invalid status")
        else:
            self.streets[heading] = status

    def get_streets(self):
        return self.streets

    def get_location(self):
        return self.location

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
        return "UNKNOWN" not in labels and "UNDRIVEN" not in labels


class MapGraph:
    """
    This object is a graph representing the set of all intersections found by 
    the bot (vertices) and the streets connecting each of those intersection (edges)
    """
    GRAPHICS_SIZE = 40

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
        prev_inters.set_connection(heading, "DRIVEN")
        inters = self.get_intersection(location)
        if inters == None:
            inters = Intersection(location, heading)
            self.graph[inters] = []
        inters.set_connection(self.invert_heading(heading), "DRIVEN")
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
        self.get_intersection(location).set_connection(heading, "NONE")

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
            if inters.check_connection(heading) not in ["UNKNOWN", "NONE"]:
                raise Exception("Expected intersection is missing, aborting")
            self.no_connection(location, heading)
        heading = (start_head + dir_map[direction] * (round(angle / 45) - 1)) % 8
        if inters.check_connection(heading) == "NONE":
            raise Exception("Nonexisting road found. Aborting")
        if inters.check_connection(heading) != "DRIVEN":
            inters.set_connection(heading, "UNDRIVEN")


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

    def __iter__(self):
        """
        Define an iterator over a MapGraph as an iterator over the dictionary
        mapping vertices to other vertices in the graph
        """
        return iter(self.graph)
    
