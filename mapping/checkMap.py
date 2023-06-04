""" This module contains a series of functions used to check the consistency of 
the map as the robot for ME/CS/EE 129 Spring '23 drives a map, either exploring 
or driving to a goal.

Authors: Edward Speer, Garrett Knuf
Date: 6/3/23
"""

import constants as const


def check_head(direction, graph, location, heading, orig_heading):
    """ Checks to make sure a turned angle was consistent with the map, 
    and if it isn't, corrects the heading to a consistent heading.
    """
    inters = graph.get_intersection(location)
    if inters.check_connection(heading) == const.NNE:
        print("Norman can't do math >:| Wrong angle, f***!")
        increment = const.dirMap[direction][1]
        orig_heading = (orig_heading + increment) % 8
        while inters.check__connection(orig_heading) == const.NNE:
            orig_heading = (orig_heading + increment) % 8
        heading = orig_heading
    return heading


def check_end(sensor, graph, location, heading):
    """ Checks the street exploration status of the road at the far end of an 
    intersection when the robot arrives at an intersection, check for 
    consistency, and updates the intersection state in the graph.
    """
    if sensor.read() == (0, 0, 0):
        if (graph.get_intersection(location).check_connection(heading) 
            not in [const.UNK, const.NNE]):
            raise Exception("Expected road missing! Aborting")
        graph.no_connection(location, heading)
    else:
        inter = graph.get_intersection(location)
        if inter.check_connection(heading) == const.NNE:
            raise Exception("Road detected where none exists!")
        if inter.check_connection(heading) != const.DRV:
            graph.get_intersection(location).set_connection(heading, const.UND)
