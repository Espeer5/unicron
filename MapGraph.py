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

    def __init__(self, location, curr_heading):
        self.location = location
        self.streets = dict.fromkeys(range(7), "UNKNOWN")
        self.streets[(curr_heading + 4) % 8] = "DRIVEN"
    
    def set_connection(self, heading, status):
        """ Set the label of a certain direction in the intersection 
            object.

            Arguments: heading - the direction to set a label for
                       status - the label to set the heading to
        """
        if status not in self.CONDITIONS:
            raise Exception("Intersection.set_connection: Invalid status")

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
        self.graph = {Intersection(location, heading):[]}

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
        self.graph[inters].append(prev_inters)
        self.graph[prev_inters].append(inters)

    def no_connection(self, location, heading):
        """
        Marks that a street doesn't exist in a certain direction for an 
        intersection in the graph

        Arguments: location: the current bot location
                    heading: the heading for which no street exists
        """
        self.graph[curr].set_connection(heading, "NONE")

    def contains(self, location):
        """
        Returns true if an intersection location is in the map, false otherwise
        """
        for intersection in self.graph:
            if intersection.get_location() == location:
                return True
        return False

    def is_complete(self):
        for intersection in self.graph:
            streets = intersection.get_streets()
            for street in streets:
                if streets[street] in ["UNKNOWN", "UNDRIVEN"]:
                    return false
        return len(self.graph) != 0 

    def get_intersection(self, location):
        for intersection in self.graph:
            if location == intersection.get_location():
                return intersection
        return None
