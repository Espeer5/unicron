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
import pickle
from mapping.planning import Djikstra
import mapping.planning as pln

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
            #if input("Show map? (y/n): ").upper() == "Y":
            #    tool.show()
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
                complete(graph)

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


def manual_djik(driveSys, sensor, location):
    """Use Djikstra's algorithm to find the shortest path to a specified
    location in a predetermined map and then follows the path
    """
    # Initial conditions (robot needs to be set at origin)
    heading = 0
    location = (0, 0)
    prev_loc = (0, 0)

    dirMap = {"L":("LEFT", 1), "R": ("RIGHT", -1), "S": "STRAIGHT"}

    # load the map from file
    filename = "map1.pickle"
    print("Loading the map from %s..." % filename)
    with open(filename, 'rb') as file:
        graph = pickle.load(file)
        tool = Visualizer(graph)
        djik = Djikstra(graph, (0, 0))

        while True:
            # get next location to drive to
            cmd = input("Enter coordinates to drive to: ")
            dest = (cmd.split(",")[0], cmd.split(",")[1])
            print("Driving to (" + dest[0] + ", " + dest[1] + ")...")

            # run algorithm and determine best path to travel
            djik.reset(dest)
            path = djik.gen_path(location)
            print("Path: " + str(path))

            # follow the path
            for i in range(len(path)):
                # orient robot to optimal heading
                direction = act.to_head(heading, path[i])
                while heading != path[i]:
                    angle = abs(act.exec_turn(driveSys, sensor, direction))
                    heading = (heading + dirMap[direction[0]][1] * angle / 45) % 8
                time.sleep(0.2)
                # drive to next intersection
                act.line_follow(driveSys, sensor)
                location = (location[0] + const.heading_map[heading][0],
                            location[1] + const.heading_map[heading][1])
                time.sleep(0.2)
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

    dirMap = {"L":("LEFT", 1), "R": ("RIGHT", -1), "S": "STRAIGHT"}

    while True:
        if act.line_follow(driveSys, sensor) == const.SUCCESS:
            time.sleep(0.2)
            prev_loc = location
            location = (location[0] + const.heading_map[heading][0],
                        location[1] + const.heading_map[heading][1])
            if graph == None:
                graph = MapGraph(location, heading)
                tool = Visualizer(graph)
                djik = Djikstra(graph, (0, 0))
            else:
                graph.driven_connection(prev_loc, location, heading)
            if sensor.read() == (0, 0, 0):
                graph.no_connection(location, heading)
            else:
                inter = graph.get_intersection(location)
                if inter.check_connection(heading) != "DRIVEN":
                    graph.get_intersection(location).set_connection(heading, "UNDRIVEN")


            dest = pln.find_unexplored(graph, location)
            print(dest)
            if dest == None:
                act.exec_turn(driveSys, sensor, ['LEFT', 'RIGHT'][random.randrange(2)])
                continue
            djik.reset(dest)
            pth = djik.gen_path(location)
            next_loc = None
            while next_loc != dest:
                if heading != pth[0]:
                    direc = act.to_head(heading, pth[0])
                    if (heading + 4) % 8 == pth[0]:
                        direc = act.l_r_unex(graph.get_intersection(location), heading)
                    while heading != pth[0]:
                        ang = abs(act.exec_turn(driveSys, sensor, direc))
                        graph.markoff(location, ang, heading, direc[0])
                        heading = (heading + dirMap[direc[0]][1] * ang / 45) % 8
                next_loc = (location[0] + const.heading_map[heading][0], 
                        location[1] + const.heading_map[heading][1])
                if next_loc == dest:
                    break
                act.line_follow(driveSys, sensor) 
                prev_loc = location
                location = (location[0] + const.heading_map[heading][0],
                        location[1] + const.heading_map[heading][1])
                graph.driven_connection(prev_loc, location, heading)
                if sensor.read() == (0, 0, 0):
                    graph.no_connection(location, heading)
                else:
                    inter = graph.get_intersection(location)
                    if inter.check_connection(heading) != "DRIVEN":
                        graph.get_intersection(location).set_connection(heading, "UNDRIVEN")
                if dest != None:
                    djik.reset(dest)
                    pth = djik.gen_path(location)
            if graph.is_complete():
                complete(graph, tool)

