"""
This file contains a set of functions which may be run to execute different 
behaviors of the robot for ME/CS/EE 129 Team Unicron. Each of these 
behaviors employs a drivesystem and linesensor to implement a certain
behavior. These behaviors were previously in behavior.py but are no longer
needed so exist in this deprecated file.

Authors: Edward Speer, Garrett Knuf
Date: 5/28/23
"""

# Imports
import time
import pigpio
from mapping.MapGraph import MapGraph
from mapping.MapGraph import complete, unb_head
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

def begin_behavior(func):
    # Initialize hardware
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connect to pigpio unicron!")
        sys.exit(0)
    print("GPIO ready...")

    # Instantiate hardware objects
    driveSys = DriveSystem(io, const.L_MOTOR_PINS, \
                                const.R_MOTOR_PINS, \
                                const.PWM_FREQ)
    IRSense = LineSensor(io, const.IR_PINS)
    ultraSense = ProximitySensor(io)
    ARG_MAP = {
    herd: (driveSys, ultraSense), 
    wall_follow: (driveSys, ultraSense, proportional_wall_follow)
    }
    try:
        func(*ARG_MAP[func])
    except KeyboardInterrupt:
        ultraSense.shutdown()
        driveSys.stop()
        io.stop()


def navigate(driveSys, sensor, tool, graph, location, heading):
    """ Assuming the robot is driving on a grid of some sort, executes 
        line following except when intersections are found, in which 
        cases the bot executes alternating left and right turns, 
        beginning with left.

        Inputs: driveSys: a drivesystem object for robot motor control
                sensor: A linesensor object for IR sensing
    """
    if input("Show map? (y/n): ").upper() == "Y":
        tool.show()
    direction = input("Direction (L/R/S)?: ")
    while direction.upper() != "S":
        while direction.upper() not in const.dirMap:
            direction = input("Invalid. Please specify a valid direction (L/R/S): ")
        angle = abs(act.exec_turn(driveSys, sensor, const.dirMap[direction.upper()][0]))
        print(angle)
        graph.markoff(location, angle, heading, direction.upper())
        heading = (heading + const.dirMap[direction.upper()][1] * angle / 45) % 8
        direction = input("Direction (L/R/S)?: ")
    return (graph, location, heading)


def herd(driveSys, sensor):
    """ Robot can be guided along a path by obstancle avoidance """
    #last_trigger_time = 0
    while True:
        #now = time.time()
        #if now - last_trigger_time > 0.05:
            #sensor.trigger()
            #last_trigger_time = time.time()
        reading = sensor.read()
        threshold = 0.2

        if reading[1] <= threshold / 2:
            driveSys.drive("BACKWARDS")
        elif reading[1] <= threshold:
            driveSys.stop()
        elif reading[0] <= threshold and reading[2] > threshold:
            driveSys.drive("TURN", "RIGHT")
        elif reading[0] > threshold and reading[2] <= threshold:
            driveSys.drive("TURN", "LEFT")
        elif reading[0] <= threshold and reading[2] <= threshold:
            if reading[0] > reading[2]:
                driveSys.drive("TURN", "LEFT")
            else:
                driveSys.drive("TURN", "RIGHT")
        else:
            driveSys.drive("STRAIGHT")


def wall_follow(driveSys, sensor, correcter):
    """ Robot follows along a wall using ultrasound sensors.
        The side of the wall it is following along must be specified as well
        as a function pointer to handle the drive system error correction """
    # get side of wall it should follow
    wall_loc = input("Follow a 'LEFT' or 'RIGHT' wall: ").upper()

    if wall_loc.upper() != "LEFT" and wall_loc.upper() != "RIGHT":
        raise Exception("Invalid wall location (must be LEFT or RIGHT)")

    sensor_mappings = {"LEFT": 0, "CENTER": 1, "RIGHT": 2}
    last_trigger_time = 0
    target_dist = const.WALL_FOLLOW_DIST

    # let ultrasounds calibrate with intial position
    sensor.trigger()
    time.sleep(const.US_DELAY)

    while True:
        # trigger ultrasound over intervals
        now = time.time()
        if now - last_trigger_time > const.US_DELAY:
            sensor.trigger()
            last_trigger_time = time.time()

        # get latest ultrasound reading and calculate error
        reading = sensor.read()[sensor_mappings[wall_loc.upper()]]
        error = reading - target_dist
        print(reading)

        # stop robot if front sensor gets too close to wall
        stop_threshold = const.US_THRESHOLD
        if sensor.read()[sensor_mappings["CENTER"]] < stop_threshold:
            driveSys.stop()

        # throw error if robot error is too large
        if abs(error) > const.US_THRESHOLD:
            raise Exception("Robot moved too far away from wall")

        # call the corresponding function to make drive system adjust
        correcter(driveSys, wall_loc, error)


def discrete_wall_follow(driveSys, wall_loc, error):
    """ drive train adjustment based on error and discrete drive settings """
    error_correction = {0.02: "STRAIGHT", 0.1: "VEER"}
    direction = "LEFT"
    if wall_loc == "LEFT":
        if error < 0:
            direction = "RIGHT"
    else:
        if error > 0:
            direction = "RIGHT"

    for key in error_correction.keys():
        if abs(error) < key:
            driveSys.drive(error_correction[key], direction)
            break


def proportional_wall_follow(driveSys, wall_loc, error):
    """ drive train adjustment based on error and constant of
        proportionality """
    prop_const = const.WALL_FOLLOW_PROP
    if wall_loc == "LEFT":
        prop_const *= -1
    left_speed = min(const.MODES["STRAIGHT"][None][0] + error * prop_const, 255)
    right_speed = max(const.MODES["STRAIGHT"][None][1] + error * prop_const, -255)
    driveSys.pwm(left_speed, right_speed)


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

def explore(driveSys, sensor, mode, stepping, step):
    """ This function is an abstraction of the general format of the bots
    exploration behaviors, which uses a functional pointer to access the required 
    behavior executed at intersections.
    """
    #Behavioral options for exploration
    EXPL_MODES = {"MANUAL": navigate, "AUTO": auto_explore, "DJIK": auto_djik}

    heading = 0
    location = (0, 0)
    graph = None
    prev_loc = (0, 0)
    tool = None
    djik = None
    while True:
        if act.line_follow(driveSys, sensor) == const.SUCCESS:
            time.sleep(.2)
            prev_loc = location
            location = (location[0] + const.heading_map[heading][0], 
                        location[1] + const.heading_map[heading][1])
            if graph == None:
                graph, tool, djik = pln.init_plan(location, heading)
            else:
                graph.driven_connection(prev_loc, location, heading)
            check_end(sensor, graph, location, heading)
            graph, location, heading = EXPL_MODES[mode](driveSys, sensor, tool, graph, location, heading, djik)
            if graph.is_complete():
                complete(graph, tool)
