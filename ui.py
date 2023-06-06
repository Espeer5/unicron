"""
This module contains the functions responsible for running the terminal
based user interface for robot control in ME/CS/EE 129 Spring '23. Asks users 
for an input command, then sets a series of shared flags with the robot control 
thread to initiate the behavior of the robot.
"""

# Authors: Edward Speer, Garrett Knuf
# Date: 5/21/23

import threading
import ctypes
from behavior.master import *


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
            if cmd == "save":
                filen = input("Enter a filename to save to")
                sigs.append(filen)
            else:
                sigs.append(None)
            return sigs
        elif cmd[0:4] == "goal":
            if len(cmd[5:].split(",")) != 2:
                print("Invalid goal format")
            else:
                sigs = CMD_DICT["goal"]
                goal = cmd[5:] 
                print("goal is " + goal)
                sigs.append(goal)
            return sigs
        else:
            print("Invalid command")


def ui_cmp():
    try:
        map_num = None
        flags = [False for _ in range(10)]
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
    