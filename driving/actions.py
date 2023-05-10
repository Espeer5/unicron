"""This module contains the funcitons for individual actions which may be 
completed by the robot in ME/CS/EE 129 Spring '23. These actions may be 
concatenated together by a behavior function to produce a behavior of the bot

Authors: Edward Speer, Garrett Knuf
Date: 5/8/23
"""

from sensing.filters import InterDetector, LRDetector, NextRoadDetector
import constants as const
import time

def line_follow(driveSys, sensor):
    """ This behavior of the robot causes the bot to begin following a 
        tape line on the ground, filtering the signal to avoid noise 
        as necessary.

        Inputs: driveSys: a DriveSystem object for motor control
                sensor: a LineSensor object to be filtered for line
                        sensing
    """
    LR_DET_RESPONSE = {-1: (driveSys.drive, ["TURN", "LEFT"]),
                        0: (driveSys.drive, ["STRAIGHT"]),
                        1: (driveSys.drive, ["TURN", "RIGHT"])}
    ids = InterDetector(sensor, const.INTER_T)
    lr = LRDetector(sensor, const.LR_T)
    start_time = time.time()
    while True:
        reading = sensor.read()
        ids.update()
        if reading == (1, 1, 1) and ids.check() and (time.time() - start_time) >= .5:
            # Drive forward to place wheels on intersection
            driveSys.drive("STRAIGHT")
            time.sleep(const.PULLUP_T)
            driveSys.stop()
            return const.SUCCESS
        lr.update()
	    # robot is entirely off the line
        if reading == (0, 0, 0):
            lr_rd = lr.get()
            resp = LR_DET_RESPONSE[lr_rd]
            resp[0](*resp[1])
        elif reading in const.FEEDBACK_TABLE: 
            driveSys.drive(const.FEEDBACK_TABLE.get(reading)[0], \
	        const.FEEDBACK_TABLE.get(reading)[1])


def to_head(heading, next_h, graph, location):
    """Computes the direction to turn to achieve a certain heading

    Arguments: heading - the current bot heading
               next_h - the desired next heading of the bot
    """
    if (heading + 4) % 8 == next_h:
        return l_r_unex(graph.get_intersection(location), heading)
    if  next_h in [(heading + i) % 8 for i in range(5)]:
        return "LEFT"
    else: 
        return "RIGHT"

def l_r_unex(inter, heading):
    """Determines whether turning left or right is better for exploring"""
    if const.UND in [inter.check_connection((heading + i) % 8) for i in range(4)] or const.UNK in [inter.check_connection((heading + i) % 8) for i in range(4)]:
        return "LEFT"
    else:
        return "RIGHT"


def calculate_angle(direction, tm):
    """Calculates the approximate angle turned by the robot for turning 
    in the given direction for the given amount of time"""
    battery_life = const.BATT_LIFE
    val = tm * battery_life
    if direction == "LEFT":
        if val > 1.35:
            return 180
        elif val > 1:
            return 135
        elif val > 0.6:
            return 90
        else:
            return 45
    elif direction == "RIGHT":
        if val > 1.35:
            return -180
        elif val > 1.07:
            return -135
        elif val > .65:
            return -90
        else:
            return - 45
    raise Exception("Invalid direction!")


def exec_turn(driveSys, sensor, direction):
    """Executes a turn of the robot until the next line is found
       using a nextroad detector, and reports the approximate angle
       the bot has turned

       Inputs: driveSys: drive system object for motor control
               sensor: linesensor object for IR input
               direction: Direction of turn (l/r)
    """
    start_time = time.time()

    #Perform a short kick to overcome resistance
    driveSys.kick(direction)

    edgeDetector = NextRoadDetector(sensor, const.NR_T, direction)
    while not edgeDetector.found_road():
        driveSys.drive("SPIN", direction)
    # first sensor has crossed the line

    # wait for center sensor to cross line for timing
    centerDetector = NextRoadDetector(sensor, const.NR_T, "CENTER")
    while not centerDetector.found_road():
        pass
    driveSys.stop()

    #Calculate the angle turned from the total time
    end_time = time.time()
    tm = end_time - start_time
    print(calculate_angle(direction, tm))
    return calculate_angle(direction, tm)

def explore_turn(driveSys, sensor, direction, graph, location, heading):
    ang = abs(exec_turn(driveSys, sensor, direction))
    time.sleep(.2)
    graph.markoff(location, ang, heading, direction[0])
    heading = (heading + const.dirMap[direction[0]][1] * ang / 45) % 8
    return heading

def path_follow(driveSys, sensor, path, location, heading, graph):
    for i in range(len(path)):
        # orient robot to optimal heading
        direction = to_head(heading, path[i], graph, location)
        while heading != path[i]:
            angle = abs(exec_turn(driveSys, sensor, direction))
            heading = (heading + const.dirMap[direction[0]][1] * angle / 45) % 8
        time.sleep(0.2)
        # drive to next intersection
        line_follow(driveSys, sensor)
        location = (location[0] + const.heading_map[heading][0],
                    location[1] + const.heading_map[heading][1])
        time.sleep(0.2)
    return (location, heading)

def check_end(sensor, graph, location, heading):
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
    prev_loc = None
    for i in range(len(path)):
        # orient robot to optimal heading
        direction = to_head(heading, path[i], graph, location)
        while heading != path[i]:
            heading = explore_turn(driveSys, sensor, direction, graph, location, heading)
        # drive to next intersection if it is not the dest
        if i != len(path) - 1:
            line_follow(driveSys, sensor)
            prev_loc = location
            location = (location[0] + const.heading_map[heading][0],
                        location[1] + const.heading_map[heading][1])
            graph.driven_connection(prev_loc, location, heading)
            check_end(sensor, graph, location, heading)
        time.sleep(0.2)
    return (location, heading)



def past_end():
    """
    A function which can be used as a function pointer which simply raises an 
    exception in the case where the robot attempts to drive straight where there
    is no road.
    """
    raise Exception("Attempted to drive straight where there is no road")
