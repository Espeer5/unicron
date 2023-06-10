"""This module contains utility functios needed across multiple threads
for sending/receiving messages from the robot UI

Authors: Edward Speer, Garrett Knuf
Date: 6/6/23
"""

import textwrap

MESSAGE_MEM = 4

def post(message, out):
    """Posts the given message to the GUI at the location indicated by the out 
    array
    """
    out[2].append(message)
    out[0].itemconfig(out[1], text=get_messages(out[2]))


def get_messages(messages):
    """ Formats the messages array into a string which may then be displayed in
    the ouput space of the GUI.
    """
    if len(messages) > MESSAGE_MEM:
        messages = messages[-MESSAGE_MEM:]
    str = ""
    for mess in messages:
        str += f"-> {textwrap.fill(mess, width=22)}\n"
    return str

def get_resp(responses, out, resp_flag):
    """Waits for a user to enter a response to a query into the entry box in the
    gui, then inputs the response into the shared responses array for use by 
    whichever thread requested the information from the user
    """
    try:
        while not resp_flag[0]:
            continue
        resp_flag[0] = False
        return responses.pop()
    except IndexError:
        post("Please Provide Response", out)


def init_state(out, responses, resp_flag, state):
    """Forces the user to manually input heading and direction prior to starting
    up any behavior of the bot
    """
    post("Input starting location: ", out)
    loc = get_resp(responses, out, resp_flag)
    post("Input starting heading: ", out)
    head = get_resp(responses, out, resp_flag)
    loc_arr = loc.split(',')
    state[0] = (int(loc_arr[0]), int(loc_arr[1]))
    state[1] = int(head)
    post("Ready to begin", out)


def set_state(state, location, heading):
    """ Sets the state variable which is shared between the UI, Robot, and 
    ros threads so that the location determined by the UI and robot threads may 
    be published via ros.
    """
    state[0] = location
    state[1] = heading


def set_flags_to(flags, target):
    """ Iterates through the shared flags array setting the flag data at each 
    index so that the robot may receive commands in the flags array from either 
    the UI thread or the ROS thread.
    """
    for i in range(len(flags)):
        flags[i] = target[i]
