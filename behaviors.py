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
from mapping.MapGraph import MapGraph
from mapping.graphics import Visualizer
import sys
import random
import constants as const
import driving.actions as act
from mapping.planning import Djikstra
import mapping.planning as pln
import pickle


def complete(graph, viz):
    """If the passed in graph has been fully emplored, saves the map to a 
    pickle file specified by the user, informs user the map is complete, and 
    exits."""
    filename = input("Enter a file name to save map to: ")
    with open(filename, 'wb') as filen:
        pickle.dump(graph, filen)
    print("Map Complete")
    viz.show()
    sys.exit(0)


def explore_turn(driveSys, sensor, direction, graph, location, heading):
    """Executes a turn around an intersection by the robot while updating the 
    intersection graph with the observed streets and ensuring self consistency 
    of the graph.
    """
    ang = abs(act.exec_turn(driveSys, sensor, direction))
    time.sleep(.2)
    graph.markoff(location, ang, heading, direction[0])
    heading = (heading + const.dirMap[direction[0]][1] * ang / 45) % 8
    return heading


def path_follow(driveSys, sensor, path, location, heading, graph):
    """Causes the robot to follow the path generated from a Djikstra instance to a 
    certain location, by turning to the approriate heading and following the 
    strees located at that heading from each intersection until the bot is facing the target.
    """
    for i in range(len(path)):
        # orient robot to optimal heading
        direction = act.to_head(heading, path[i], graph, location)
        while heading != path[i]:
            angle = abs(act.exec_turn(driveSys, sensor, direction))
            heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
        time.sleep(0.2)
        # drive to next intersection
        act.line_follow(driveSys, sensor)
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


def path_explore(driveSys, sensor, path, graph, location, heading):
    """Follows a path to a target location produced by a Djikstra instance while also updating
    the MapGraph with all information observed along the way.
    """
    prev_loc = None
    for i in range(len(path)):
        # orient robot to optimal heading
        direction = act.to_head(heading, path[i], graph, location)
        while heading != path[i]:
            heading = explore_turn(driveSys, sensor, direction, graph, location, heading)
        # drive to next intersection if it is not the dest
        if i != len(path) - 1:
            act.line_follow(driveSys, sensor)
            prev_loc = location
            location = (location[0] + const.heading_map[heading][0],
                        location[1] + const.heading_map[heading][1])
            graph.driven_connection(prev_loc, location, heading)
            check_end(sensor, graph, location, heading)
        time.sleep(0.2)
    return (location, heading)


def navigate(driveSys, sensor):
    """ Assuming the robot is driving on a grid of some sort, executes 
        line following except when intersections are found, in which 
        cases the bot executes alternating left and right turns, 
        beginning with left.

        Inputs: driveSys: a drivesystem object for robot motor control
                sensor: A linesensor object for IR sensing
    """
    # Maps user input to driveSys instructions
    dirMap = {"L":("LEFT", 1), "R": ("RIGHT", -1), "S": "STRAIGHT"}
    # Heading of robot encoded as integer 0-7
    heading = 0
    # Robot location stored as (longitude, latitude)
    location = (0, 0)
    graph = None
    prev_loc = (0, 0)
    tool = None
    while True:
        if act.line_follow(driveSys, sensor) == const.SUCCESS:
            prev_loc = location
            location = (location[0] + const.heading_map[heading][0], 
                        location[1] + const.heading_map[heading][1])
            if graph == None:
                graph = MapGraph(location, heading)
                tool = Visualizer(graph)
            else:
                graph.driven_connection(prev_loc, location, heading)
            if sensor.read() == (0, 0, 0):
                graph.no_connection(location, heading)
            else:
                inter = graph.get_intersection(location)
                if inter.check_connection(heading) != "DRIVEN":
                    graph.get_intersection(location).set_connection(heading, "UNDRIVEN")
            if input("Show map? (y/n): ").upper() == "Y":
                tool.show()
            direction = input("Direction (L/R/S)?: ")
            while direction.upper() != "S":
                while direction.upper() not in dirMap:
                    direction = input("Invalid. Please specify a valid direction (L/R/S): ")
                angle = abs(act.exec_turn(driveSys, sensor, dirMap[direction.upper()][0]))
                print(angle)
                graph.markoff(location, angle, heading, direction.upper())
                if graph.is_complete():
                    complete(graph, tool)
                heading = (heading + dirMap[direction.upper()][1] * angle / 45) % 8
                direction = input("Direction (L/R/S)?: ")


def auto_explore(driveSys, sensor):
    """
    Allows the robot to autonomously explore a maze of roads until 
    it has mapped the entire thing, then display a graphical 
    representation of the map

    Arguments: driveSys: driveSys object for motor control
               sensor: linesensor object for IR sensing
    """
    # Heading of robot as integer 0-7
    heading = 0
    # Robot location stored as (longitude, latitude)
    location = (0, 0)
    prev_loc = (0, 0)

    graph = None
    tool = None

    while True:
        # follow line until intersection is found
        # add intersection to map
        if act.line_follow(driveSys, sensor) == const.SUCCESS:
            prev_loc = location
            location = (location[0] + const.heading_map[heading][0],
                        location[1] + const.heading_map[heading][1])
            if graph == None:
                graph = MapGraph(location, heading)
                tool = Visualizer(graph)
            else:
                graph.driven_connection(prev_loc, location, heading)
            # explore intersection and determine shape
            intersection = graph.get_intersection(location)
            time.sleep(0.2)
            while const.UNK in intersection.get_streets().values():
                angle = act.exec_turn(driveSys, sensor, "RIGHT")
                graph.markoff(location, angle, heading, "R")
                heading = (heading + (-angle / 45)) % 8
                time.sleep(0.2)

            if graph.is_complete():
                complete(graph, tool)

            # turn to undriven road if it exists
            if const.UND in intersection.get_streets().values():
                while intersection.get_streets()[heading] != const.UND:
                    angle = act.exec_turn(driveSys, sensor, "RIGHT")
                    graph.markoff(location, angle, heading, "R")
                    heading = (heading + (-angle / 45)) % 8
                    time.sleep(0.2)
            else:
                num_roads = 0
                for label in intersection.get_streets().values():
                    if label == const.DRV:
                        num_roads += 1
                num_turns = 1
                if num_roads != 1:
                    num_turns = random.randrange(1, num_roads + 1)
                for i in range(num_turns):
                    angle = act.exec_turn(driveSys, sensor, "RIGHT")
                    graph.markoff(location, angle, heading, "R")
                    heading = (heading + (-angle / 45)) % 8
                    time.sleep(0.2)


def manual_djik(driveSys, sensor):
    """Use Djikstra's algorithm to find the shortest path to a specified
    location in a predetermined map and then follows the path
    """
    # Initial conditions (robot needs to be set at origin)
    heading = 0
    location = (0, 0)

    # load the map from file
    graph = pln.from_pickle()
    tool = Visualizer(graph)
    djik = Djikstra(graph, (0, 0))
    tool.show()
    graph.print_graph()
    
    # Drive forward to orient the bot within the map
    act.line_follow(driveSys, sensor)
    location = (location[0] + const.heading_map[heading][0],
                location[1] + const.heading_map[heading][1])

    # Navigate to user requested locations until aborted
    while True:
        # get next location to drive to
        cmd = input("Enter coordinates to drive to: ")
        dest = (int(cmd.split(",")[0]), int(cmd.split(",")[1]))
        if not graph.contains(dest):
            print("Location does not exist!")
            continue
        print("Driving to (" + str(dest[0]) + ", " + str(dest[1]) + ")...")

        # run algorithm and determine best path to travel
        djik.reset(dest)
        path = djik.gen_path(location)
        print("Path: " + str(path))

        # follow the path
        location, heading = path_follow(driveSys, sensor, path, location, heading, graph)

        print("Robot has successfully reached " + str(dest))   

        
def auto_djik(driveSys, sensor):
    """Uses Djikstra's algorithm to intelligently explore the map by taking
    efficient paths to unexplored locations
    """

    heading = 0
    location = (0, 0)
    prev_loc = (0, 0)
    graph = None
    djik = None

    while True:
        print(["LET ME COOK", "IM COOKIN", "I BE SHAKIN BAKIN", "YAH-YAHHH"][random.randrange(4)])
        if act.line_follow(driveSys, sensor) == const.SUCCESS:
            time.sleep(0.2)
            prev_loc = location
            location = (location[0] + const.heading_map[heading][0],
                        location[1] + const.heading_map[heading][1])
            if graph == None:
                graph, tool, djik = pln.init_plan(location, heading)
            else:
                graph.driven_connection(prev_loc, location, heading)
            check_end(sensor, graph, location, heading)
            l_heads = [(heading + i) % 8 for i in range(4)]
            r_heads = [(heading - i) % 8 for i in range(4)]
            directi = None
            if const.UNK in [graph.get_intersection(location).check_connection(head) for head in l_heads]:
                directi = "LEFT"
            elif const.UNK in [graph.get_intersection(location).check_connection(head) for head in r_heads]:
                directi = "RIGHT"
            if directi != None:
                orig_heading = heading
                while heading != (orig_heading + 4) % 8:
                    heading = explore_turn(driveSys, sensor, directi, graph, location, heading)
                continue
            p_head = pln.unx_dir(graph.get_intersection(location))
            if p_head != None:
                if p_head == 0:
                    continue
                heading = explore_turn(driveSys, sensor, act.to_head(heading, p_head, graph, location), graph, location, heading)
                continue

            if graph.is_complete():
                complete(graph, tool)
            
            dest = pln.find_unexplored(graph, location)
            if dest == None:
                direc = pln.unx_dir(graph.get_intersection(location))
                if direc == None:
                    direc = heading
                heading = explore_turn(driveSys, sensor, act.to_head(heading, direc, graph, location), graph, location, heading)
                continue
            djik.reset(dest)
            pth = djik.gen_path(location)
            location, heading = path_explore(driveSys, sensor, pth, graph, location, heading)

