"""This module contains utility functios needed across multiple threads
for sending/receiving messages from the robot UI

Authors: Edward Speer, Garrett Knuf
Date: 6/6/23
"""

import textwrap

MESSAGE_MEM = 10


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
    try:
        while not resp_flag:
            continue
        resp_flag[0] = False
        return responses.pop()
    except IndexError:
        post("Please Provide Response", out)

