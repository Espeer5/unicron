""" This module contains the functions which tell the robot which direction to 
turn upon reaching a new intersection when driving to complete a command
specified to the ui. These functions should be called by the master behavior 
in between successive line follows in order to decide and turn to the 
correct next intersection.

Authors: Edward Speer, Garrett Knuf
Date: 6/3/23
"""

import time 
import constants as const
import driving.actions as act
import mapping.checkMap as checks
import mapping.planning as pln 
from math import dist
from mapping.MapGraph import unb_head
from interface.ui_util import post


def explore_turn(driveSys, IRSensor, ultraSense, direction, graph, location, 
                 heading, out, responses, resp_flag):
    """Executes a turn around an intersection by the robot while updating the 
    intersection graph with the observed streets and ensuring self consistency 
    of the graph.
    """
    #Turn until a street is found, then stop to kill momentum
    ang = abs(act.exec_turn(driveSys, IRSensor, direction))
    time.sleep(.2)

    #Update the heading from the angle turned, and ensure consistent with  map.
    orig_head = heading
    heading = (heading + const.dirMap[direction[0]][1] * ang / 45) % 8
    heading, ang = checks.check_head(direction, graph, location, heading, 
                                     orig_head, ang, out, responses, resp_flag)
    post("angle: " + str(ang), out)

    #Update the graph based on where a street was found
    graph.markoff(location, ang, orig_head, direction[0])

    #Check for blockages on the faced street
    if graph != None:
         act.center_block(ultraSense, location, heading, graph, out)
    return heading


def auto_inters(driveSys, sensor, graph, heading, location, ultraSense, out, 
                responses, resp_flag):
    """ A general algorithm for exploring an intersection. This can be used to
    explore an unexplored intersection, and then the calling function must 
    determine how to reach the next unexplored intesection.
    """
    #Get the possible headings for turning each direction
    l_heads = [(heading + i) % 8 for i in range(4)]
    r_heads = [(heading - i) % 8 for i in range(4)]

    #If there are unkown headings, investigate them
    directi = None
    if const.UNK in [graph.get_intersection(location).check_connection(head) 
                     for head in l_heads]:
        directi = "LEFT"
    elif const.UNK in [graph.get_intersection(location).check_connection(head) 
                       for head in r_heads]:
        directi = "RIGHT"
    if directi != None:
        orig_heading = heading
        while heading != (orig_heading + 4) % 8:
            heading = explore_turn(driveSys, sensor, ultraSense, directi, graph,
                                    location, heading, out, responses, 
                                    resp_flag)
        return (graph, location, heading, True)

    #If there are undriven streets, turn to them and drive them
    p_head = pln.unx_dir(graph.get_intersection(location))
    if p_head != None:
        if graph.get_intersection(location).check_blockage(p_head) == const.UNB:
            while heading != p_head:
                heading = explore_turn(driveSys, sensor, ultraSense, 
                                       pln.to_head(heading, p_head, graph, 
                                                   location), 
                                                   graph, location, heading, 
                                                   out, responses, resp_flag)
            return (graph, location, heading, True)
    
    #Otherwise, return that no exploration was done yet
    return (graph, location, heading, False)

        
def auto_djik(driveSys, IRSensor, ultraSense, path, graph, location, heading, 
              djik, prev_loc, out, responses, resp_flag, state):
    """Uses Djikstra's algorithm to intelligently explore the map by taking
    efficient paths to unexplored locations
    """
    #If there is already a path stored, follow it
    if path != []:
        path_elem = path.pop(0)
        direction = pln.to_head(heading, path_elem, graph, location)
        num_turns = 0
        while heading != path_elem:
            if num_turns > 2:
                post("Bad angle was read, resetting", out)
                init_state(out, responses, resp_flag, state)
                location = state[0]
                heading = state[1]
                prev_loc = (location[0] - const.heading_map[heading][0], 
                location[1] - const.heading_map[heading][1])
                set_state(state, location, heading)
                break
            angle = abs(act.exec_turn(driveSys, IRSensor, direction))
            heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
            num_turns += 1
        time.sleep(.02)
        return (path, graph, location, heading)
    
    #Otherwise, check if the current intersection needs to be explored, and 
    # if so, explore it
    graph, location, heading, explored = auto_inters(driveSys, IRSensor, 
                                                         graph, heading, 
                                                         location, ultraSense, 
                                                         out, responses, 
                                                         resp_flag)
    if explored:
        return (path, graph, location, heading)
         
    #Otherwise, use Djikstra to find an efficient path to an unexplored location
    post("Recalculating djik algorithm...", out)
    dest = pln.find_unexplored(graph, location, [])
    if dest == None:
        direc = pln.unx_dir(graph.get_intersection(location))

        if direc == None:
            direc = unb_head(graph, location)

        while heading != direc:
            heading = explore_turn(driveSys, IRSensor, ultraSense, 
                                   pln.to_head(heading, direc, graph, location),
                                     graph, location, heading, out, responses, 
                                     resp_flag)
        return (path, graph, location, heading)
    djik.reset(dest)
    path = djik.gen_path(location)
    
    #If no unexplored location found to explore, drive on some unblocked heading
    if path == []:
        direc = unb_head(graph, location)
        while heading != direc:
            heading = explore_turn(driveSys, IRSensor, ultraSense, 
                                   pln.to_head(heading, direc, graph, location),
                                     graph, location, heading, out, responses, 
                                     resp_flag)
        return (path, graph, location, heading)

    #Drive along the path generate by the above cases
    path_elem = path.pop(0)
    direction = pln.to_head(heading, path_elem, graph, location)
    while heading != path_elem:
        angle = abs(act.exec_turn(driveSys, IRSensor, direction))
        heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
    time.sleep(.02)
    return (path, graph, location, heading)


def manual_djik(driveSys, IRSensor, ultraSense, path, heading, graph, location, djik, 
                flags, out, responses, resp_flag, subtarget):
    """Use Djikstra's algorithm to find the shortest path to a specified
    location in a predetermined map and then follows the path
    """
    done = False   
    # Get new destination
    cmd = flags[const.DATA]
    dest = (int(cmd.split(",")[0]), int(cmd.split(",")[1]))
    if dest == location:
        done = True
        print("Goal reached @ " + str(dest) + "!")
        return (path, heading, graph, location, done, subtarget)
    
    # Determine is destination is unreachable
    if graph.is_complete():
        graph.clear_blockages()
        if graph.is_complete() and not graph.contains(dest):
            post("Destination " + str(dest) + " unreachable", out)
            return (path, heading, graph, location, done, subtarget) 

    # If a path is found to destination, follow it
    if graph.contains(dest):
        djik.reset(dest)
        path = djik.gen_path(location)
        subtarget = dest
        if path == []:
            unexplored_inters = graph.unexp_inters()
            stuck = True
            for inter in unexplored_inters:
                djik.reset(inter)
                path = djik.gen_path(location)
                if path != []:
                    stuck = False
                    break
            if stuck:
                graph.clear_blockages()
                post("Stuck, clearing blockages", out)
                if graph.unexp_inters() == []:
                    post("Stuck", out)
                    return (path, heading, graph, location, done, subtarget)
                subtarget = None

    # Otherwise find nearest known intersection to target and drive there
    elif subtarget == None:
        subtarget = pln.closest_subtarget(graph, location, heading, dest, djik)
        print("RECALCULATING SUBTARGET to " + str(subtarget))
        # If still no subtarget is found the explore current intersection
        if subtarget == None:
            direction = pln.l_r_s_to_target(graph.get_intersection(location),
                                            heading, dest)
            print("LRS: " + direction)
            if direction != "STRAIGHT":
                heading = explore_turn(driveSys, IRSensor, ultraSense, direction,
                                        graph, location, heading, out, responses,
                                        resp_flag)
            subtarget = None
            return (path, heading, graph, location, done, subtarget)
        # Otherwise calcuate path to subtarget with djikstra
        djik.reset(subtarget)
        path = djik.gen_path(location)  
    post("Going to subtarget " + str(subtarget), out) 
    print("SUBTARGET: " + str(subtarget) + "; Currently at " + str(location))

    # If robot does know have path to destination then try to find one
    if not graph.contains(dest):
        # Recalculate every iteration to account for blockages
        djik.reset(subtarget)
        path = djik.gen_path(location)

    # If robot reaches subtarget, turn to optimal heading for destination
    if location == subtarget:
        post("Subtarget reached at " + str(subtarget), out)
        direction = pln.l_r_s_to_target(graph.get_intersection(location),
                                        heading, dest)
        print("LRS to Target: " + direction)
        if direction != "STRAIGHT":
            heading = explore_turn(driveSys, IRSensor, ultraSense, direction,
                                    graph, location, heading, out, responses,
                                    resp_flag)
        subtarget = None
        return (path, heading, graph, location, done, subtarget)

    # If path cannot be found to subtarget, reroute
    if path == []:
        subtarget = pln.closest_subtarget(graph, location, heading, dest, djik)
        # If path still cannot be found then we need to clear blockages
        if subtarget == None:
            post("Stuck! Clearing Blockages", out)
            graph.clear_blockages()
            djik.reset(subtarget)
            path = djik.gen_path(location)
            # If path still cannot be found then we are stuck
            post("Norman is stuck! No route to " + str(dest) + " can be found", out)
            done = True
            return (path, heading, graph, location, done, subtarget)
        djik.reset(subtarget)
        path = djik.gen_path(location)
        
    post("Driving to (" + str(dest[0]) + ", " + str(dest[1]) + ")...", out)

    # Follow djikstra generated path
    path_elem = path.pop(0)
    direction = pln.to_head(heading, path_elem, graph, location)
    while heading != path_elem:
        heading = explore_turn(driveSys, IRSensor, ultraSense, direction,
                                graph, location, heading, out, responses,
                                resp_flag)
    time.sleep(.02)
    return (path, heading, graph, location, done, subtarget)