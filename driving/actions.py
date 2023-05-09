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
            time.sleep(.37)
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


def to_head(heading, next_h):
    """Computes the direction to turn to achieve a certain heading

    Arguments: heading - the current bot heading
               next_h - the desired next heading of the bot
    """
    if  next_h in [(heading + i) % 8 for i in range(5)]:
        return "LEFT"
    else: 
        return "RIGHT"

def l_r_unex(inter, heading):
    """Determines whether turning left or right is better for exploring"""
    if const.UND in [inter.check_connection((heading + i) % 8) for i in range(5)] or const.UNK in [inter.check_connection((heading + i) % 8) for i in range(5)]:
        return "LEFT"
    else:
        return "RIGHT"


def calculate_angle(direction, tm):
    """Calculates the approximate angle turned by the robot for turning 
    in the given direction for the given amount of time"""
    if direction == "LEFT":
        if tm > 1.7:
            return 180
        elif tm > 1.4:
            return 135
        elif tm > 0.85:
            return 90
        else:
            return 45
    elif direction == "RIGHT":
        if tm > 1.65:
            return -180
        elif tm > 1.34:
            return -135
        elif tm > .8:
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
    return calculate_angle(direction, tm)


def past_end():
    """
    A function which can be used as a function pointer which simply raises an 
    exception in the case where the robot attempts to drive straight where there
    is no road.
    """
    raise Exception("Attempted to drive straight where there is no road")
