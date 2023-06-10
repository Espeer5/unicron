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
from mapping.MapGraph import unb_head
from interface.ui_util import post


def explore_turn(driveSys, IRSensor, ultraSense, direction, graph, location, 
                 heading, out, responses, resp_flag):
    """Executes a turn around an intersection by the robot while updating the 
    intersection graph with the observed streets and ensuring self consistency 
    of the graph.
    """
    ang = abs(act.exec_turn(driveSys, IRSensor, direction))
    time.sleep(.2)
    orig_head = heading
    heading = (heading + const.dirMap[direction[0]][1] * ang / 45) % 8
    heading, ang = checks.check_head(direction, graph, location, heading, 
                                     orig_head, ang, out, responses, resp_flag)
    post("angle: " + str(ang), out)
    graph.markoff(location, ang, orig_head, direction[0])
    if graph != None:
         act.center_block(ultraSense, location, heading, graph)
    return heading


def auto_inters(driveSys, sensor, graph, heading, location, ultraSense, out, 
                responses, resp_flag):
    """ A general algorithm for exploring an intersection. This can be used to
    explore an unexplored intersection, and then the calling function must 
    determine how to reach the next unexplored intesection.
    """
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
    return (graph, location, heading, False)

        
def auto_djik(driveSys, IRSensor, ultraSense, path, graph, location, heading, 
              djik, prev_loc, out, responses, resp_flag):
    """Uses Djikstra's algorithm to intelligently explore the map by taking
    efficient paths to unexplored locations
    """
    if path != []:
        path_elem = path.pop(0)
        direction = pln.to_head(heading, path_elem, graph, location)
        while heading != path_elem:
            angle = abs(act.exec_turn(driveSys, IRSensor, direction))
            heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
        time.sleep(.02)
        return (path, graph, location, heading)
    
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
                                   act.to_head(heading, direc, graph, location),
                                     graph, location, heading, out, responses, 
                                     resp_flag)
        return (path, graph, location, heading)
    djik.reset(dest)
    path = djik.gen_path(location)
    if path == []:
        direc = unb_head(graph, location)
        while heading != direc:
            heading = explore_turn(driveSys, IRSensor, ultraSense, 
                                   act.to_head(heading, direc, graph, location),
                                     graph, location, heading, out, responses, 
                                     resp_flag)
        return (path, graph, location, heading)
    path_elem = path.pop(0)
    direction = pln.to_head(heading, path_elem, graph, location)
    while heading != path_elem:
        angle = abs(act.exec_turn(driveSys, IRSensor, direction))
        heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
    time.sleep(.02)
    return (path, graph, location, heading)


def manual_djik(driveSys, IRSensor, path, heading, graph, location, djik, cmd, 
                flags, out):
    """Use Djikstra's algorithm to find the shortest path to a specified
    location in a predetermined map and then follows the path
    """
    done = False
    if path != []:
        if len(path) == 1:
            post("Driving Last leg", out)
            flags[const.REPLAN] = True
        path_elem = path.pop(0)
        direction = act.to_head(heading, path_elem, graph, location)
        while heading != path_elem:
            angle = abs(act.exec_turn(driveSys, IRSensor, direction))
            heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
        time.sleep(.02)
        return (path, heading, graph, location, done)
    else:
        dest = (int(cmd.split(",")[0]), int(cmd.split(",")[1]))
        if dest == location:
            done = True
            return (path, heading, graph, location, done) 
        if cmd == None: # or dest == location:
            raise Exception("Norman will not navigate to where he already is")
        djik.reset(dest)
        path = djik.gen_path(location)
        post("Driving to (" + str(dest[0]) + ", " + str(dest[1]) + ")...", out)
        path_elem = path.pop(0)
        direction = act.to_head(heading, path_elem, graph, location)
        while heading != path_elem:
            angle = abs(act.exec_turn(driveSys, IRSensor, direction))
            heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
        time.sleep(.02)
        return (path, heading, graph, location, done)
    path_elem = path.pop(0)
    direction = pln.to_head(heading, path_elem, graph, location)
    while heading != path_elem:
        angle = abs(act.exec_turn(driveSys, IRSensor, direction))
        heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
    time.sleep(.02)
    return (path, heading, graph, location, done)