"""This module contains the funcitons for individual actions which may be 
completed by the robot in ME/CS/EE 129 Spring '23. These actions may be 
concatenated together by a behavior function to produce a behavior of the bot

Authors: Edward Speer, Garrett Knuf
Date: 5/8/23
"""

from sensing.filters import InterDetector, LRDetector, NextRoadDetector
from mapping.graphics import Visualizer
import constants as const
import time

def line_follow(driveSys, IRSense, ultraSense, tool):
    """ This behavior of the robot causes the bot to begin following a 
        tape line on the ground, filtering the signal to avoid noise 
        as necessary. It returns SUCCESS if a line if successfully followed,
        but it will stop the robot return FAILURE if an object is detected
        in the robots path

        Inputs: driveSys: a DriveSystem object for motor control
                IRSense: a LineSensor object to be filtered for line
                        sensing
                ultraSense: a ProximitySensor to detect object in the
                            robot's path
                tool: visualizer to be updated upon successful follow
    """
    LR_DET_RESPONSE = {-1: (driveSys.drive, ["TURN", "LEFT"]),
                        0: (past_end, []),
                        1: (driveSys.drive, ["TURN", "RIGHT"])}
    Ntime = time.time()
    ids = InterDetector(IRSense, const.INTER_T, Ntime)
    lr = LRDetector(IRSense, const.LR_T, Ntime)
    start_time = time.time()

    while True:
        # check if obstacle in robot path
        dist = ultraSense.read()[1]
        if dist <= const.OBJECT_COLLISION_DIST:
            driveSys.stop()
            return const.FAILURE
        # perform line following based on IR readings
        reading = IRSense.read()
        Ntime = time.time()
        ids.update(time.time())
        if reading == (1, 1, 1) and ids.check(Ntime) and (Ntime - start_time) >= .5:
            driveSys.stop()
            time.sleep(1.2)
            if tool != None:
                tool.show()
            return const.SUCCESS
        lr.update(time.time())
	    # robot is entirely off the line
        if reading == (0, 0, 0):
            lr_rd = lr.get(time.time())
            resp = LR_DET_RESPONSE[lr_rd]
            resp[0](*resp[1])
        elif reading in const.FEEDBACK_TABLE: 
            driveSys.drive(const.FEEDBACK_TABLE.get(reading)[0], \
	        const.FEEDBACK_TABLE.get(reading)[1])

def adv_line_follow(driveSys, IRSensor, ultraSense, tool, location, heading, graph):
    """ This behavior of the robot performs an advacned line follow, however,
        if detects an object immediately in its path with the ultrasound
        sensors, it performs a 180 degree U-turn. If it gets stuck on a road
        for more than 3 U-turns, it will wait until an block blocking the
        road is removed
    """
    num_Uturns = 0
    while line_follow(driveSys, IRSensor, ultraSense, tool) != const.SUCCESS:
        heading, location = exec_Uturn(driveSys, IRSensor, location, heading)
        num_Uturns += 1
        if num_Uturns >= 2:
            while ultraSense.read()[1] < 0.35:
                pass # wait until obstacle is removed
            num_Uturns = 0

    prev_loc = location
    location = (location[0] + const.heading_map[heading][0], 
                location[1] + const.heading_map[heading][1])
    if graph != None:
        find_blocked_streets(ultraSense, location, heading, graph)

    if tool != None:
        tool.show()
    return location, prev_loc, heading

def pullup(driveSys):
    """ Robot drive forward to center itself in an intersection """
    driveSys.drive("STRAIGHT")
    time.sleep(const.PULLUP_T)
    driveSys.stop()
    time.sleep(.5)

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
    l_list = [inter.check_connection((heading + i) % 8) for i in range(4)]
    r_list = [inter.check_connection((heading + i) % 8) for i in range(4)]
    if const.UND in l_list:
        if const.UND in r_list:
            if r_list.index(const.UND) < l_list.index(const.UND):
                return "RIGHT"
    return "LEFT"

def l_r_unb(inter, heading):
    """Determines whether left or right is better for finding an unblocked intersection"""
    l_list = [inter.check_connection((heading + i) % 8) for i in range(4)]
    r_list = [inter.check_connection((heading + i) % 8) for i in range(4)]
    if const.UNB in l_list:
        if const.UNB in r_list:
            if r_list.index(const.UNB) < l_list.index(const.UNB):
                return "RIGHT"
    return "LEFT"


def calculate_angle(direction, tm):
    """Calculates the approximate angle turned by the robot for turning 
    in the given direction for the given amount of time"""
    battery_life = const.BATT_LIFE
    val = tm * battery_life
    if direction == "LEFT":
        if val > 1.42:
            return 180
        elif val > 1:
            return 135
        elif val > 0.6:
            return 90
        else:
            return 45
    elif direction == "RIGHT":
        if val > 1.42:
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

    edgeDetector = NextRoadDetector(sensor, const.NR_T, direction, start_time)
    while not edgeDetector.found_road():
        driveSys.drive("SPIN", direction)
    # first sensor has crossed the line

    # wait for center sensor to cross line for timing
    centerDetector = NextRoadDetector(sensor, const.NR_T, "CENTER", time.time())
    while not centerDetector.found_road():
        continue
    driveSys.stop()

    #Calculate the angle turned from the total time
    end_time = time.time()
    tm = end_time - start_time
    time.sleep(.5)
    return calculate_angle(direction, tm)


def exec_Uturn(driveSys, IRSensor, location, heading):
    """
    A function that executes a 180 degree turn for the robot in the middle of a
    road. It assumes that the angle turned was 180 degrees regardless of the
    angle calculated. 
    """
    print("Skirrrrrrt! Flipping a U-ey")
    ang = abs(exec_turn(driveSys, IRSensor, "RIGHT"))
    time.sleep(.2)
    heading = (heading + 4) % 8  # assume 180 degree angle
    prev_loc = location
    if ang != 180:
        print(ang)
        print("GASP! The robot read a bad angle!!!")
        prev_loc = (location[0] + const.heading_map[heading][0], 
                         location[1] + const.heading_map[heading][1])
    return heading, prev_loc


def past_end():
    """
    A function which can be used as a function pointer which simply raises an 
    exception in the case where the robot attempts to drive straight where there
    is no road.
    """
    raise Exception("Attempted to drive straight where there is no road")


def find_blocked_streets(ultraSense, location, heading, graph):
    """
    Search for blocked street ahead only if street ahead exists. Updates the graph
    and returns a boolean whether it found any blocked streets.
    """

    time.sleep(1) # der der der

    # allowable distance until object blocks a street
    threshold = 0.5
    if heading % 2 != 0:
        threshold = 0.7

    

    if graph != None:
        inters = graph.get_intersection(location)
        if inters != None:
            # filter ultrasound readings because the sensors suck
            readings = []
            filter_steps = 5
            for i in range(filter_steps):
                time.sleep(0.06)
                readings.append(ultraSense.read())
            
            left_sensor_bad = False
            center_sensor_bad = False
            right_sensor_bad = False

            for i in range(len(readings)):
                if readings[i][0] > threshold:
                    left_sensor_bad = True
                if readings[i][1] > threshold:
                    center_sensor_bad = True
                if readings[i][2] > threshold:
                    right_sensor_bad = True

            # center sensor
            next_location = (location[0] + const.heading_map[heading][0],
                             location[1] + const.heading_map[heading][1])
            if (graph.contains(next_location) or inters.get_streets()[heading] == const.UND or \
                inters.get_streets()[heading] == const.DRV) and not center_sensor_bad:
                graph.block_connection(location, next_location, heading)

            # left sensor
            next_location = (location[0] + const.heading_map[(heading+2)%8][0],
                             location[1] + const.heading_map[(heading+2)%8][1])
            if (graph.contains(next_location) or inters.get_streets()[(heading+2)%8] == const.UND or \
                inters.get_streets()[(heading+2)%8] == const.DRV) and not left_sensor_bad:
                graph.block_connection(location, next_location, (heading+2)%8)

            # right sensor
            next_location = (location[0] + const.heading_map[(heading-2)%8][0],
                             location[1] + const.heading_map[(heading-2)%8][1])
            if (graph.contains(next_location) or inters.get_streets()[(heading-2)%8] == const.UND or \
                inters.get_streets()[(heading-2)%8] == const.DRV) and not right_sensor_bad:
                graph.block_connection(location, next_location, (heading-2)%8)


            # if graph.contains(next_location) or inters.get_streets()[heading] == const.UND or \
            #     inters.get_streets()[heading] == const.DRV:
            #     if not left_sensor_bad:
            #         if graph.get_intersection(location).get_blockages()[(heading+2)%8] != const.BLK:
            #             next_location = (location[0] + const.heading_map[(heading+2)%8][0],
            #                              location[1] + const.heading_map[(heading+2)%8][1])
            #             graph.block_connection(location, next_location, (heading+2)%8)
            #     if not center_sensor_bad:
            #         if graph.get_intersection(location).get_blockages()[heading] != const.BLK:
            #             next_location = (location[0] + const.heading_map[heading][0],
            #                              location[1] + const.heading_map[heading][1])
            #             graph.block_connection(location, next_location, heading)
            #     if not right_sensor_bad:  
            #         if graph.get_intersection(location).get_blockages()[(heading-2)%8] != const.BLK:
            #             next_location = (location[0] + const.heading_map[(heading-2)%8][0],
            #                              location[1] + const.heading_map[(heading-2)%8][1])
            #             graph.block_connection(location, next_location, (heading-2)%8)

