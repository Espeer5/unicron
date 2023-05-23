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
from mapping.MapGraph import complete
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


def explore(driveSys, sensor, mode):
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


def navigate(driveSys, sensor, tool, graph, location, heading, djik):
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


def auto_inters(driveSys, sensor, graph, heading, location):
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
            heading = explore_turn(driveSys, sensor, directi, graph, location, heading)
        return (graph, location, heading, True)

    #If there are undriven streets, turn to them and drive them
    p_head = pln.unx_dir(graph.get_intersection(location))
    if p_head != None:
        heading = explore_turn(driveSys, sensor, act.to_head(heading, p_head, graph, location), graph, location, heading)
        return (graph, location, heading, True)
    else:
        return (graph, location, heading, False)


def auto_explore(driveSys, sensor, tool, graph, location, heading, djik):
    """
    Allows the robot to autonomously explore a maze of roads until 
    it has mapped the entire thing, then display a graphical 
    representation of the map

    Arguments: driveSys: driveSys object for motor control
               sensor: linesensor object for IR sensing
    """
    graph, location, heading, explored = auto_inters(driveSys, sensor, graph, heading, location)
    if explored:
        return (graph, location, heading)
    else:
        heading = explore_turn(driveSys, sensor, ["LEFT", "RIGHT"][random.randrange(0, 2)], graph, location, heading)
        return (graph, location, heading)

        
def auto_djik(driveSys, sensor, tool, graph, location, heading, djik):
    """Uses Djikstra's algorithm to intelligently explore the map by taking
    efficient paths to unexplored locations
    """
    graph, location, heading, explored = auto_inters(driveSys, sensor, graph, heading, location)
    if explored:
        return (graph, location, heading)
    else:
        #Otherwise, use Djikstra to find an efficient path to an unexplored location
        dest = pln.find_unexplored(graph, location)
        if dest == None:
            direc = pln.unx_dir(graph.get_intersection(location))
            if direc == None:
                direc = heading
            heading = explore_turn(driveSys, sensor, act.to_head(heading, direc, graph, location), graph, location, heading)
            return (graph, location, heading)
        djik.reset(dest)
        pth = djik.gen_path(location)
        location, heading = path_explore(driveSys, sensor, pth, graph, location, heading)

        return (graph, location, heading)


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

