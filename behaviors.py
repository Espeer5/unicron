"""
This file contains a set of functions which may be run to execute different 
behaviors of the robot for ME/CS/EE 129 Team Unicron. Each of these 
behaviors employs a drivesystem and linesensor to implement a certain
behavior.

Authors: Edward Speer, Garrett Knuf
Date: 4/24/23
"""

# Imports
import time
import pigpio
from mapping.MapGraph import MapGraph
from mapping.MapGraph import complete, unb_head, unk_dir
from mapping.graphics import Visualizer
import sys
import random
import constants as const
import driving.actions as act
from driving.driveSystem import DriveSystem
from sensing.linesensor import LineSensor
from mapping.planning import Djikstra
import mapping.planning as pln
import pickle
from sensing.proximitysensor import ProximitySensor


def explore_turn(driveSys, IRSensor, ultraSense, direction, graph, location, heading):
    """Executes a turn around an intersection by the robot while updating the 
    intersection graph with the observed streets and ensuring self consistency 
    of the graph.
    """
    ang = abs(act.exec_turn(driveSys, IRSensor, direction))
    time.sleep(.2)
    print("angle: " + str(ang))
    graph.markoff(location, ang, heading, direction[0])
    heading = (heading + const.dirMap[direction[0]][1] * ang / 45) % 8
    if graph != None:
         act.find_blocked_streets(ultraSense, location, heading, graph)
    return heading


def path_follow(driveSys, IRSensor, path, location, heading, graph):
    """Causes the robot to follow the path generated from a Djikstra instance to a 
    certain location, by turning to the approriate heading and following the 
    strees located at that heading from each intersection until the bot is facing the target.
    """
    for i in range(len(path)):
        # orient robot to optimal heading
        direction = act.to_head(heading, path[i], graph, location)
        while heading != path[i]:
            # turn to desired angle
            angle = abs(act.exec_turn(driveSys, IRSensor, direction))
            heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
        time.sleep(0.2)
        # drive to next intersection
        act.line_follow(driveSys, IRSensor)
        location = (location[0] + const.heading_map[heading][0],
                    location[1] + const.heading_map[heading][1])
        time.sleep(0.2)
    return (location, heading)


def check_end(sensor, graph, location, heading):
    """ Checks the street exploration status of the road at the far end of an intersection when the
    robot arrives at an intersection, check for consistency, and updates the intersection state 
    in the graph.
    """
    if sensor.read() == (0, 0, 0):
        if graph.get_intersection(location).check_connection(heading) not in [const.UNK, const.NNE]:
            raise Exception("Expected road missing! Aborting")
        graph.no_connection(location, heading)
    else:
        inter = graph.get_intersection(location)
        if inter.check_connection(heading) == const.NNE:
            raise Exception("Road detected where none exists!")
        if inter.check_connection(heading) != const.DRV:
            graph.get_intersection(location).set_connection(heading, const.UND)


def auto_inters(driveSys, sensor, graph, heading, location, ultraSense):
    """ A general algorithm for exploring an intersection. This can be used to
    explore an unexplored intersection, and then the calling function must 
    determine how to reach the next unexplored intesection.
    """
    l_heads = [(heading + i) % 8 for i in range(4)]
    r_heads = [(heading - i) % 8 for i in range(4)]
    #If there are unkown headings, investigate them
    directi = None
    if const.UNK in [graph.get_intersection(location).check_connection(head) for head in l_heads]:
        directi = "LEFT"
    elif const.UNK in [graph.get_intersection(location).check_connection(head) for head in r_heads]:
        directi = "RIGHT"
    if directi != None:
        orig_heading = heading
        while heading != (orig_heading + 4) % 8:
            heading = explore_turn(driveSys, sensor, ultraSense, directi, graph, location, heading)
        return (graph, location, heading, True)

    #If there are undriven streets, turn to them and drive them
    p_head = pln.unx_dir(graph.get_intersection(location))
    if p_head != None:
        if graph.get_intersection(location).check_blockage(p_head) == const.UNB:
            while heading != p_head:
                heading = explore_turn(driveSys, sensor, ultraSense, act.to_head(heading, p_head, graph, location), graph, location, heading)
            return (graph, location, heading, True)
    return (graph, location, heading, False)

        
def auto_djik(driveSys, IRSensor, ultraSense, path, graph, location, heading, djik, prev_loc, explorable):
    """Uses Djikstra's algorithm to intelligently explore the map by taking
    efficient paths to unexplored locations
    """
    if path != [] and explorable:
        path_elem = path.pop()
        direction = act.to_head(heading, path_elem, graph, location)
        while heading != path_elem:
            angle = abs(act.exec_turn(driveSys, IRSensor, direction))
            heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
        time.sleep(.02)
        return (path, graph, location, heading)
    
    if explorable:
        graph, location, heading, explored = auto_inters(driveSys, IRSensor, graph, heading, location, ultraSense)
        if explored and explorable:
            return (path, graph, location, heading)
         
    #Otherwise, use Djikstra to find an efficient path to an unexplored location
    print("Recalculating djik algorithm...")
    dest = pln.find_unexplored(graph, location)
    if dest == None:
        direc = pln.unx_dir(graph.get_intersection(location))
        #if graph.get_intersection(location).check_blockage(direc):

        print("DIREC1 " + str(direc))
        if direc == None:
            direc = unb_head(graph, location)
            print("DIREC2 " + str(direc))

        while heading != direc:
            heading = explore_turn(driveSys, IRSensor, ultraSense, act.to_head(heading, direc, graph, location), graph, location, heading)
        return (path, graph, location, heading)
    djik.reset(dest)
    path = djik.gen_path(location)
    if path == []:
        direc = unb_head(graph, location)
        while heading != direc:
            heading = explore_turn(driveSys, IRSensor, ultraSense, act.to_head(heading, direc, graph, location), graph, location, heading)
        return (path, graph, location, heading)
    path_elem = path.pop()
    direction = act.to_head(heading, path_elem, graph, location)
    while heading != path_elem:
        angle = abs(act.exec_turn(driveSys, IRSensor, direction))
        heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
    time.sleep(.02)
    return (path, graph, location, heading)


def manual_djik(driveSys, IRSensor, path, heading, graph, location, djik, cmd, flags):
    """Use Djikstra's algorithm to find the shortest path to a specified
    location in a predetermined map and then follows the path
    """
    if path != []:
        if len(path) == 1:
            print("Last leg")
            flags[8] = None
        path_elem = path.pop()
        direction = act.to_head(heading, path_elem, graph, location)
        while heading != path_elem:
            angle = abs(act.exec_turn(driveSys, IRSensor, direction))
            heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
        time.sleep(.02)
        return (path, heading, graph, location)
    else:
        dest = (int(cmd.split(",")[0]), int(cmd.split(",")[1]))
        if cmd == None or dest == location:
            raise Exception("Norman will not navigate to where he already is >:(")
        djik.reset(dest)
        path = djik.gen_path(location)
        print("Driving to (" + str(dest[0]) + ", " + str(dest[1]) + ")...")
        path_elem = path.pop()
        direction = act.to_head(heading, path_elem, graph, location)
        while heading != path_elem:
            angle = abs(act.exec_turn(driveSys, IRSensor, direction))
            heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
        time.sleep(.02)
        return (path, heading, graph, location)  


def master(flags, map_num=None):
    # Initialize hardware
    io = pigpio.pi()
    if not io.connected:
        sys.exit(0)
    # Instantiate hardware objects
    driveSys = DriveSystem(io, const.L_MOTOR_PINS, \
                                const.R_MOTOR_PINS, \
                                const.PWM_FREQ)
    IRSensor = LineSensor(io, const.IR_PINS)
    ultraSense = ProximitySensor(io)
    
    while((flags[0], flags[1]) == (False, False)):
        time.sleep(2)
        continue
    graph = None
    if map_num != None:
        graph = pln.from_pickle(map_num)
    if graph == None and flags[1]:
        raise Exception("Norman needs a map to navigate >:(")
    if flags[0] and flags[1]:
        raise Exception("Norman cannot explore and navigate at the same time >:(")
    location = (0, 0)
    heading = 0 
    prev_loc = (0, 0)
    tool = None
    if graph != None:
        tool = Visualizer(graph)
    djik = None 
    if graph != None:
        djik = Djikstra(graph, (0, 0))
    path = []
    explorable = True

    try:
        while True:
            if flags[6]:
                break
            if flags[7]:
                if graph != None:
                    graph.clear_blockages()
            if flags[4]:
                complete(graph, flags[8])
                flags[4] = False
            if flags[5]:
                if tool == None:
                    print("Norman has no map to display >:(")
                else:
                    tool.show()

            if graph != None:
                if graph.get_intersection(location).get_blockages()[heading] != const.BLK:
                    location, prev_loc, heading = act.adv_line_follow(driveSys, IRSensor, ultraSense, tool, location, heading, graph)
                    act.pullup(driveSys)
                    explorable = True
                else:
                    #print("Robot Status: Heading " + str(heading) + " at Location " + str(location))
                    #explorable = False
                    path = []
                    #pass
            else:
                location, prev_loc, heading = act.adv_line_follow(driveSys, IRSensor, ultraSense, tool, location, heading, graph)
                graph, tool, djik = pln.init_plan(location, heading)
                print("Normstorm Navigation Enabled")
                act.find_blocked_streets(ultraSense, location, heading, graph)
                act.pullup(driveSys)
                explorable = True
            
            act.find_blocked_streets(ultraSense, location, heading, graph)
            graph.driven_connection(prev_loc, location, heading)
            # we can assume no 45 degree roads exist upon approaching intersection
            graph.no_connection(location, (heading + 3) % 8)
            graph.no_connection(location, (heading + 5) % 8)
            check_end(IRSensor, graph, location, heading)

            if flags[2]:
                while not flags[3]:
                    continue
                flags[3] = False
            time.sleep(.2)
            if flags[1]:
                while flags[8] == None:
                    continue
                path, heading, graph, location = manual_djik(driveSys, IRSensor, path, heading, graph, location, djik, flags[8], flags)
                continue
            if flags[0]:
                if graph.is_complete():
                    print("Map Fully Explored!")
                    flags = [True, False, True, False, False, False, False, False] # pause
                else:
                    path, graph, location, heading = auto_djik(driveSys, IRSensor, ultraSense, path, graph, location, heading, djik, prev_loc, explorable)
                continue
        print("Shutting down")
        ultraSense.shutdown()
        driveSys.stop()
        io.stop()
    except KeyboardInterrupt:
        print("Shutting down")
        ultraSense.shutdown()
        driveSys.stop()
        io.stop()