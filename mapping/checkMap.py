""" This module contains a series of functions used to check the consistency of 
the map as the robot for ME/CS/EE 129 Spring '23 drives a map, either exploring 
or driving to a goal.

Authors: Edward Speer, Garrett Knuf
Date: 6/3/23
"""

import constants as const
from interface.ui_util import post, get_resp, init_state


def check_head(direction, graph, location, heading, orig_heading, ang, out, 
               responses, resp_flag):
    """ Checks to make sure a turned angle was consistent with the map, 
    and if it isn't, corrects the heading to a consistent heading.
    """
    inters = graph.get_intersection(location)
    if inters.check_connection(heading) == const.NNE:
        post("Norman can't do math >:| Wrong angle, f***!", out)
        increment = const.dirMap[direction[0]][1]
        orig_heading = (orig_heading + increment) % 8
        from_orig = 1
        while inters.check_connection(orig_heading) == const.NNE:
            from_orig += 1
            orig_heading = (orig_heading + increment) % 8
        heading = orig_heading
        ang = from_orig * 45
        if inters.check_connection(heading) == const.UNK:
            post(f"Correcting error... am I facing heading {heading} (y/n)?", out)
            if(get_resp(responses, out, resp_flag).upper() == "N"):
                post("Input true heading: ", out)
                heading = int(get_resp(responses, out, resp_flag))
                while orig_heading != heading:
                    orig_heading = (orig_heading + increment) % 8
                    from_orig += 1
                ang = from_orig * 45
    return (heading, ang)


def check_end(sensor, graph, location, heading, out, responses, resp_flag, state):
    """ Checks the street exploration status of the road at the far end of an 
    intersection when the robot arrives at an intersection, check for 
    consistency, and updates the intersection state in the graph.
    """
    if sensor.read() == (0, 0, 0):
        if graph.get_intersection(location).check_connection(heading) not in [const.UNK, const.NNE]:
            post("Incorrect angle measured! Help!", out)
            init_state(out, responses, resp_flag, state)
            location = state[0]
            heading = state[1]
            prev_loc = (location[0] - const.heading_map[heading][0], 
            location[1] - const.heading_map[heading][1])
            set_state(state, location, heading)
        graph.no_connection(location, heading)
    else:
        inter = graph.get_intersection(location)
        if inter.check_connection(heading) == const.NNE:
            post("Incorrect angle measured! Help!", out)
            init_state(out, responses, resp_flag, state)
            location = state[0]
            heading = state[1]
            prev_loc = (location[0] - const.heading_map[heading][0], 
            location[1] - const.heading_map[heading][1])
            set_state(state, location, heading)
        if inter.check_connection(heading) != const.DRV:
            graph.get_intersection(location).set_connection(heading, const.UND)
