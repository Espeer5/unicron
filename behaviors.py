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
    while True:
        if line_follow(driveSys, sensor) == SUCCESS:
            prev_loc = location
            location = (location[0] + heading_map[heading][0], 
                        location[1] + heading_map[heading][1])
            if graph == None:
                graph = MapGraph(location, heading)
            else:
                graph.driven_connection(prev_loc, location, heading)
            direction = input("Direction (L/R/S)?: ")
            while direction.upper() != "S":
                while direction.upper() not in dirMap:
                    direction = input("Invalid. Please specify a \
                                      \ valid direction (L/R/S): ")

                angle = exec_turn(driveSys, sensor, dirMap[direction.upper()][0])
                driveSys.stop()
                heading = (heading + dirMap[direction.upper()][1] * angle / 45) % 8
                direction = input("Direction (L/R/S)?: )")
            driveSys.drive("STRAIGHT")
            time.sleep(.5)
            print(f"Location: {location}, Heading: {heading}")


def line_follow(driveSys, sensor):
    """ This behavior of the robot causes the bot to begin following a 
        tape line on the ground, filtering the signal to avoid noise 
        as necessary.

        Inputs: driveSys: a DriveSystem object for motor control
                sensor: a LineSensor object to be filtered for line
                        sensing
    """
    LR_DET_RESPONSE = {-1: (driveSys.drive, ["HOOK", "LEFT"]),
                        0: (driveSys.stop, []),
                        1: (driveSys.drive, ["HOOK", "RIGHT"])} 

    ids = InterDetector(sensor, .04)
    lr = LRDetector(sensor, .01)
    while True:
        reading = sensor.read()
        ids.update()
        if reading == (1, 1, 1) and ids.check():
            # Drive forward to place wheels on intersection
            driveSys.drive("STRAIGHT")
            time.sleep(.2)
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
    nrd = NextRoadDetector(sensor, 0.0055, direction)
    while not nrd.found_road():
        driveSys.drive("SPIN", direction) 
    # first sensor has crossed line
    # wait for center sensor to cross line for timing
    while sensor.read()[1] == 0:
        pass
    end_time = time.time()
    omega = 190
    # Approximate angle turned using angular velocity
    angle = (omega * (end_time - start_time))
    # Round the angle to the nearest 45 degrees
    angle = round(angle / 45) * 45
    return angle
    
