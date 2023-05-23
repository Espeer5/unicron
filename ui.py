"""
This module contains the functions responsible for running the terminal
based user interface for robot control in ME/CS/EE 129 Spring '23

"""

# Authors: Edward Speer, Garrett Knuf
# Date: 5/21/23

import threading
import ctypes
from behaviors import *

def get_input_simple():
    """
    This function gets the user input from the terminal and returns the
    appropriate flags to the main UI loop to indicate what actions should be 
    executed.

    Returns: (Active, Running), where active indicates if the robot should 
    be executing behavior, and running indicates whether the program should keep 
    running or exit
    """
    while True:
        cmd = input("Enter a command: ").lower()
        if cmd == "run":
            return (True, True)
        elif cmd == "stop":
            return (False, True)
        if cmd == "quit":
            return (False, False)
        else:
            print("Invalid command!")

def start_normstorm(func):
    """Exectues the passed in function in a new thread which controls the 
    robot.
    """
    robot_thread = threading.Thread(name="RobotThread", \
                                 target=begin_behavior,
                                 args=[func])
    robot_thread.start()
    return robot_thread


def kill_robot(robot_thread):
    """Upon the command from the UI, stops all robot activities and closes down 
    threads associated with robot activity.
    """
    ctypes.pythonapi.PyThreadState_SetAsyncExc(
        ctypes.c_long(robot_thread.ident),
        ctypes.py_object(KeyboardInterrupt))
    print("Waiting for robot thread to return")
    robot_thread.join()
    print("Robot thread returned")
    return None


def ui_simple(func):
    """Takes in input from the user and causes the robot to execute the desired 
    behavior in a new thread.
    """
    robot_thread = None
    try:
        running = True 
        active = False
        while True:
            prev = active
            active, running = get_input()
            if prev == active:
                if not running:
                    break
                else:
                    print("Already Normstormin' like that")
                    continue
            if not active:
                robot_thread = kill_robot(robot_thread)
                if not running:
                    break
                continue
            if active:
                robot_thread = start_normstorm(func)
                continue
    except KeyboardInterrupt:
        if robot_thread != None:
            kill_robot(robot_thread)
    print("Exiting")


def cmp_input():
    CMD_DICT = {
        "pause": [True, False, True, False, False, False, False],
        "explore": [True, False, -1, -1, False, -1, False],
        "goal": [False, True, -1, -1, False, -1, False],
        "show": [-1, -1, -1, -1, -1, True, False],
        "stepping": [-1, -1, True, -1, -1, -1, False],
        "step": [-1, -1, -1, True, -1, -1, False],
        "save": [-1, -1, -1, -1, True, -1, False],
        "quit": [-1, -1, -1, -1, -1, -1, True]
    }
    while True:
        cmd = input("input command: ").lower()
        if cmd in CMD_DICT:
            return CMD_DICT[cmd]
        else:
            print("Invalid command")

def ui_cmp():
    try:
        plan = input("Would you like to load a map? (y/n)").lower()
        pickle = False
        if plan == "y":
            pickle = True
        flags = [False for i in range(7)]
        robot_thread = threading.Thread(name="RobotThread", \
                                 target=master,
                                 args=[flags, pickle])
        robot_thread.start()
        while True:
            temp = cmp_input()
            for i in range(len(flags)):
                if temp[i] != -1:
                    flags[i] = temp[i]
            if flags[6] == 1:
                kill_robot()
                break
    except KeyboardInterrupt:
        if robot_thread != None:
            kill_robot(robot_thread)
    print("Exiting")
    
