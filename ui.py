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
    """Executes the passed in function in a new thread which controls the 
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
            active, running = get_input_simple()
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
    # flags: explore, navigate, stepping, step, save, show, quit, clear
    CMD_DICT = {
        "pause": [True, False, True, False, False, False, False, False, False],
        "explore": [True, False, False, True, False, -1, False, False, False],
        "goal": [False, True, False, False, False, -1, False, False, False],
        "show": [-1, -1, -1, -1, -1, True, False, False, False],
        "stepping": [-1, -1, True, -1, -1, -1, False, False, False],
        "step": [-1, -1, -1, True, -1, -1, False, False, False],
        "save": [-1, -1, -1, -1, True, -1, False, False, False],
        "quit": [-1, -1, -1, -1, -1, -1, True, -1, False],
        "clear": [-1, -1, -1, -1, -1, -1, False, True, False],
    }
    while True:
        cmd = input("input command: ").lower()
        if cmd in CMD_DICT:
            sigs = CMD_DICT[cmd]
            if cmd == "goal":
                goal = input("Input a location to drive to: ")
                sigs.append(goal)
            else:
                sigs.append(None)
            if cmd == "save":
                filen = input("Enter a filename to save to")
                sigs.append(filen)
            else:
                sigs.append(None)
            return sigs
        else:
            print("Invalid command")

def ui_cmp():
    try:
        map_num = None
        flags = [False for i in range(10)]
        robot_thread = threading.Thread(name="RobotThread", \
                                 target=master,
                                 args=[flags, map_num])
        robot_thread.start()
        print("WARNING: norman is feeling extra naughty (ง ͠° ͟ʖ ͡°)ง")
        while True:
            temp = cmp_input()
            for i in range(len(flags)):
                if temp[i] != -1:
                    flags[i] = temp[i]
            if flags[6] == 1:
                kill_robot(robot_thread)
                break
    except KeyboardInterrupt:
        if robot_thread != None:
            kill_robot(robot_thread)
    print("Exiting")
    
