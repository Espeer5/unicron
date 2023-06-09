"""
This file contains the master behavior controlling function for the robot 
for ME/CS/EE 129 Spring '23 for Team Unicron. The master function executes a 
line follow, updates state variables and map data structure, then makes a call 
to a decision making funcitonal pointer to decide how to go to the next 
intersection based on the command indicated to the user interface thread. This 
master controller should be the main controller called by the robot control 
thread.

Authors: Edward Speer, Garrett Knuf
Date: 4/24/23
"""

# Imports
import time
import pigpio
from mapping.MapGraph import complete
from mapping.graphics import Visualizer
import sys
import constants as const
import driving.actions as act
from driving.driveSystem import DriveSystem
from sensing.linesensor import LineSensor
from mapping.planning import Djikstra
import mapping.planning as pln
from sensing.proximitysensor import ProximitySensor
import mapping.checkMap as checks
from behavior.decision import *
from interface.ui_util import post


def end(ultraSense, driveSys, io):
    """ Stop all activities of the robot and ends communication with the robot 
    hardware in preparation to shutdown the robot control thread. Also ends 
    all activity from the UltraSense thread and shuts the thread down

    Arguments: ultraSense: The active ultraSense object being read by the robot
               driveSys: The motor control object being used to control the bot 
               io: The IO object being used to communicate with bot hardware 
    """
    print("Shutting down")
    ultraSense.shutdown()
    driveSys.stop()
    io.stop()
  

def master(flags, out, responses, resp_flag, map_num=None):
    """ This function interacts with the UI module to allow Norman to execute 
    different behaviors, allowing Norman to switch behaviors in between turns. 
    First executes a line follow, then decides how to turn based on which
    behavior is being executed.

    Arguments: flags: the flags array shared with the UI thread for commands 
               map_num: Optionally load a map from the pickle file with this #
    """
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
    
    while((flags[const.EXP_FLAG], flags[const.GL_FLAG]) == (False, False)):
        time.sleep(2)
        continue
    graph = None
    if map_num != None:
        graph = pln.from_pickle(map_num)
    if graph == None and flags[const.GL_FLAG]:
        raise Exception("Norman needs a map to navigate >:(")
    if flags[const.EXP_FLAG] and flags[const.GL_FLAG]:
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
    active = True

    try:
        while True:
            if flags[const.QUIT]:
                break
            if flags[const.CLEAR]:
                if graph != None:
                    graph.clear_blockages()
            if flags[const.SV_MAP]:
                complete(graph, flags[const.DATA])
                flags[const.SV_MAP] = False
            if flags[const.DISP_MAP]:
                if tool == None:
                    post("Norman has no map to display >:(", out)
                else:
                    tool.show_path(location, path)
            if graph != None:
                if (graph.get_intersection(location).get_blockages()[heading]
                    != const.BLK and active):
                    location, prev_loc, heading = act.adv_line_follow(driveSys, 
                                                                      IRSensor, 
                                                                      ultraSense, 
                                                                      tool, 
                                                                      location, 
                                                                      heading, 
                                                                      graph)
                    act.pullup(driveSys)
                else:
                    path = []
            else:
                location, prev_loc, heading = act.adv_line_follow(driveSys, 
                                                                  IRSensor, 
                                                                  ultraSense, 
                                                                  tool, 
                                                                  location, 
                                                                  heading, 
                                                                  graph)
                graph, tool, djik = pln.init_plan(location, heading)
                post("Normstorm Navigation Enabled", out)
                act.find_blocked_streets(ultraSense, location, heading, graph)
                act.pullup(driveSys)
            if prev_loc != location:
                graph.driven_connection(prev_loc, location, heading)
            # we can assume no 45 degree roads exist approaching intersection
            graph.no_connection(location, (heading + 3) % 8)
            graph.no_connection(location, (heading + 5) % 8)
            checks.check_end(IRSensor, graph, location, heading)

            if flags[const.STP_FLAG]:
                while not flags[const.STP]:
                    continue
                flags[const.STP] = False
            time.sleep(.2)
            if flags[const.GL_FLAG]:
                while flags[const.DATA] == None:
                    continue
                path, heading, graph, location, done = manual_djik(driveSys,
                                                                   IRSensor,
                                                                   path, 
                                                                   heading, 
                                                                   graph, 
                                                                   location, 
                                                                   djik, 
                                                                   flags[const.DATA], 
                                                                   flags, out, 
                                                                   responses, 
                                                                   resp_flag)
                active = not done
                continue
            elif flags[const.EXP_FLAG]:
                active = True
                if graph.is_complete():
                    post("Map Fully Explored!", out)
                    active = False
                else:
                    path, graph, location, heading = auto_djik(driveSys, 
                                                               IRSensor, 
                                                               ultraSense, 
                                                               path, 
                                                               graph, 
                                                               location, 
                                                               heading, 
                                                               djik, 
                                                               prev_loc, out)
                continue
        end(ultraSense, driveSys, io)
    except KeyboardInterrupt:
        end(ultraSense, driveSys, io)
