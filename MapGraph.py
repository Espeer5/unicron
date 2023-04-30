"""
This module contains the objects which make up the map representation
stored and tracked by the robot as it navigates mazes in ME/CS/EE 129.

Authors: Edward Speer, Garrett Knuf
Date: 4/30/23
"""


class Intersection:
    """ An intersection object maps each heading from an intersection
    to a label which describes the bot's knowledge of whether a road
    is present in any of the 8 possible bot headings

    Inputs: curr_heading - the heading which the bot first approached
    the intersection from
    """

    # Discrete set of labels available for each heading at an intersection
    CONDITIONS = ["UNKNOWN", "UNDRIVEN", "NONE"]

    def __init__(self, curr_heading):
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

    def invert_heading(heading):
        """ A simple utility which returns the opposite heading (180 degrees) given
        a heading
        """
        return (heading + 4) % 8

    def __init__(self):
        self.graph = {}

    def driven_connection(self, prev_location, location, heading):
        """
        Sets the streets for each intersection when a connection 
        between 2 intersections has been driven

        Arguments: prev_location: the last visited intersection location
                   location: the location of the current intersection
                   heading: the current heading of the bot
        """
        self.graph[prev_location].set_connection(heading, "DRIVEN")
        if curr in self.graph:
            self.graph[curr].set_connection(invert_heading(heading), "DRIVEN")
        else:
            self.graph[curr] = Intersection(heading)

    def no_connection(self, location, heading):
        """
        Marks that a street doesn't exist in a certain direction for an 
        intersection in the graph

        Arguments: location: the current bot location
                    heading: the heading for which no street exists
        """
        self.graph[curr].set_connection(heading, "NONE")

    def contains(self, intersection):
        """
        Returns true if an intersection is in the map, false otherwise
        """
        return intersection in self.graph
