"""
This file contains a set of functions which may be run to execute different 
behaviors of the robot for ME/CS/EE 129 Team Unicron. Each of these 
behaviors employs a drivesystem and linesensor to implement a certain
behavior.

Authors: Edward Speer, Garrett Knuf
Date: 4/24/23
"""

from filters import Filters, InterDetector, LRDetector, NextRoadDetector
import time
from MapGraph import MapGraph
from graphics import Visualizer
import sys
import random

# The feedback law governing steady state line following
FEEDBACK_TABLE = {(0, 1, 0): ("STRAIGHT", None),
                     (1, 1, 1): ("STRAIGHT", None), \
                     (0, 1, 1): ("TURN", "RIGHT"), \
                     (0, 0, 1): ("TURN", "RIGHT"), \
		             (1, 1, 0): ("TURN", "LEFT"), \
		             (1, 0, 0): ("TURN", "LEFT")}

# An alias for the intersection found exit condition from line follow
SUCCESS = 1

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
    # Maps the robot heading to the change in location after driving
    heading_map = {0: (0, 1), 1: (1, 1), 2: (1, 0), 3: (1, -1), 
                    4: (0, -1), 5: (-1, -1), 6: (-1, 0), 7: (-1, 1)}
    graph = None
    prev_loc = (0, 0)
    tool = None
    while True:
        if line_follow(driveSys, sensor) == SUCCESS:
            prev_loc = location
            location = (location[0] + heading_map[heading][0], 
                        location[1] + heading_map[heading][1])
            if graph == None:
                graph = MapGraph(location, heading)
                tool = Visualizer(graph)
            else:
                graph.driven_connection(prev_loc, location, heading)
            if input("Show map? (y/n): ").upper() == "Y":
                tool.show()
            direction = input("Direction (L/R/S)?: ")
            while direction.upper() != "S":
                while direction.upper() not in dirMap:
                    direction = input("Invalid. Please specify a valid direction (L/R/S): ")
                angle = exec_turn(driveSys, sensor, dirMap[direction.upper()][0])
                graph.markoff(location, angle, heading, direction.upper())
                if graph.is_complete():
                    print("Map Complete")
                    sys.exit(0)
                heading = (heading + dirMap[direction.upper()][1] * angle / 45) % 8
                direction = input("Direction (L/R/S)?: ")

def past_end():
    """
    A function which can be used as a function pointer which simply raises an 
    exception in the case where the robot attempts to drive straight where there
    is no road.
    """
    raise Exception("Attempted to drive straight where there is no road")


def line_follow(driveSys, sensor):
    """ This behavior of the robot causes the bot to begin following a 
        tape line on the ground, filtering the signal to avoid noise 
        as necessary.

        Inputs: driveSys: a DriveSystem object for motor control
                sensor: a LineSensor object to be filtered for line
                        sensing
    """
    LR_DET_RESPONSE = {-1: (driveSys.drive, ["HOOK", "LEFT"]),
                        0: (past_end, []),
                        1: (driveSys.drive, ["HOOK", "RIGHT"])} 

    ids = InterDetector(sensor, .03)
    lr = LRDetector(sensor, .01)
    start_time = time.time()
    while True:
        reading = sensor.read()
        ids.update()
        if reading == (1, 1, 1) and ids.check() and (time.time() - start_time) >= .5:
            # Drive forward to place wheels on intersection
            driveSys.drive("STRAIGHT")
            time.sleep(.35)
            driveSys.stop()
            return SUCCESS
        lr.update()
	    # robot is entirely off the line
        if reading == (0, 0, 0):
            lr_rd = lr.get()
            resp = LR_DET_RESPONSE[lr_rd]
            resp[0](*resp[1])
        elif reading in FEEDBACK_TABLE: 
            driveSys.drive(FEEDBACK_TABLE.get(reading)[0], \
	        FEEDBACK_TABLE.get(reading)[1])
    

def exec_turn(driveSys, sensor, direction):
    """Executes a turn of the robot until the next line is found
       using a nextroad detector, and reports the approximate angle
       the bot has turned

       Inputs: driveSys: drive system object for motor control
               sensor: linesensor object for IR input
               direction: Direction of turn (l/r)
    """
    start_time = time.time()
    nrd = NextRoadDetector(sensor, 0.015, direction)
    while not nrd.found_road():
        driveSys.drive("SPIN", direction) 
    # first sensor has crossed line
    # wait for center sensor to cross line for timing
    while sensor.read()[1] == 0:
        pass
    driveSys.stop()
    end_time = time.time()
    time.sleep(0.2)
    omega = 170
    # Approximate angle turned using angular velocity
    angle = (omega * (end_time - start_time))
    # adjust robot overshoot if large turn is made
    if direction.upper() == "RIGHT":
        delay = 0.11
        if angle > 115:
            delay = 0.145
            omega = 162
            if angle > 140:
                omega = 171
            if angle > 200: 
                omega = 185
            if angle > 280:
                omega = 193
        driveSys.drive("SPIN", "LEFT")
        time.sleep(delay)
        driveSys.stop()
    if direction.upper() == "LEFT":
        if angle > 250:
            omega = 165

    battery_life = 0.96
    angle = (omega * battery_life * (end_time - start_time))
    print(angle)
    # Round the angle to the nearest 45 degrees
    angle = round(angle / 45) * 45
    return angle


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
    # Maps the robot heading to the change in location after driving
    heading_map = {0: (0, 1), 1: (1, 1), 2: (1, 0), 3: (1, -1),
                   4: (0, -1), 5: (-1, -1), 6: (-1, 0), 7: (-1, 1)}

    graph = None
    tool = None

    while True:
        # follow line until intersection is found
        # add intersection to map
        if line_follow(driveSys, sensor) == SUCCESS:
            prev_loc = location
            location = (location[0] + heading_map[heading][0],
                        location[1] + heading_map[heading][1])
            if graph == None:
                graph = MapGraph(location, heading)
                tool = Visualizer(graph)
            else:
                graph.driven_connection(prev_loc, location, heading)
            # explore intersection and determine shape
            intersection = graph.get_intersection(location)
            time.sleep(0.2)
            while "UNKNOWN" in intersection.get_streets().values():
                angle = exec_turn(driveSys, sensor, "RIGHT")
                graph.markoff(location, angle, heading, "R")
                heading = (heading + (-angle / 45)) % 8
                time.sleep(0.2)

            if graph.is_complete():
                print("Map Complete")
                tool.show()
                sys.exit(0)

            # turn to undriven road if it exists
            if "UNDRIVEN" in intersection.get_streets().values():
                while intersection.get_streets()[heading] != "UNDRIVEN":
                    angle = exec_turn(driveSys, sensor, "RIGHT")
                    graph.markoff(location, angle, heading, "R")
                    heading = (heading + (-angle / 45)) % 8
                    time.sleep(0.2)
            else:
                num_roads = 0
                for label in intersection.get_streets().values():
                    if label == "DRIVEN":
                        num_roads += 1
                num_turns = 1
                if num_roads != 1:
                    num_turns = random.randrange(1, num_roads + 1)
                for i in range(num_turns):
                    angle = exec_turn(driveSys, sensor, "RIGHT")
                    graph.markoff(location, angle, heading, "R")
                    heading = (heading + (-angle / 45)) % 8
                    time.sleep(0.2)
