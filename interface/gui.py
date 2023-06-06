"""This module contains the funcitons needed to initialize and update the 
graphical user interface used for robot control in ME/CS/EE 129 Spring '23. 
The GUI allows the user to input commands to the robot, see robot status
commentary/output, and respond to robot requests for information in seperate IO
streams to avoid IO collisions between threads.

Authors: Edward Speer, Garrett Knuf
Date: 6/6/23
"""

import tkinter as tk
from PIL import ImageTk, Image
import threading
import ctypes
from behavior.master import *
from interface.ui_util import *

#GUI Window size
X_SIZE = 950
Y_SIZE = 680

#Maximum number of posted messages saved in the GUI at a time
MESSAGE_MEM = 6

def run_gui():
    """ Initializes the GUI window and populates it with the features making 
    up the control interface. Executes the main loop of the graphics window
    (and hence doesn't return)
    """
    print("Interface Running...")
    messages = []
    flags = [False for _ in range(10)]

    responses = []
    resp_flag = [False]
    
    root = tk.Tk()
    root.tk_setPalette(background="white")
    root.title("NormStorm Controller")
    root.configure(bg="white")
    root.geometry(f"{X_SIZE}x{Y_SIZE}")
    root.minsize(X_SIZE, Y_SIZE)
    root.maxsize(X_SIZE, Y_SIZE)

    top_frame = tk.Frame(root, width=X_SIZE, height=150, 
                         highlightbackground="black", highlightthickness=3)
    top_frame.pack()
    top_frame.place(anchor='center', relx=0.5, rely=.06)

    img = ImageTk.PhotoImage(Image.open("ns_tbar.png"))

    img_label = tk.Label(top_frame, image=img)
    img_label.pack()

    map_frame = tk.Frame(root, width=X_SIZE/2)
    map_frame.pack()
    map_frame.place(anchor='center', relx=.75, y=450)


    map = Image.open("map.png")
    map = map.resize((500, 400))
    map = ImageTk.PhotoImage(map)

    map_label = tk.Label(map_frame, image=map)
    map_label.pack()

    out = OSpace(root, messages)
    outs = (out[0], out[1], messages)

    robot_thread = create_rth(flags, outs, responses, resp_flag)

    cmd_entry(root, outs, flags, robot_thread)
    resp_entry(root, responses, resp_flag)

    post("WARNING: norman is feeling extra naughty (ง ͠° ͟ʖ ͡°)ง", outs)
    
    root.after(1000, lambda: update_gmap(root, map_label))
    root.protocol("WM_DELETE_WINDOW", lambda: on_close(root, robot_thread))
    try:
        root.mainloop()
    except KeyboardInterrupt:
        on_close(root, robot_thread)


def OSpace(root, messages):
    """ Creates the output space for message posting by the robot to the GUI.
    This function returns the output object shared between threads which allows 
    message posting to the GUI.
    """
    OFrame = tk.Frame(root, width=X_SIZE/4, height=400)
    OFrame.pack()
    OFrame.place(anchor='center', relx=.2, y=480)
    OCanvas = tk.Canvas(OFrame, width=X_SIZE/3, height=400)
    OCanvas.create_text(110, 30, text="Status Messages", fill="black", 
                        font="Times 20 bold underline")
    text = OCanvas.create_text(110, 50, text=get_messages(messages), 
                               fill="black", font="Times 15", anchor="n")
    OCanvas.pack()
    return (OCanvas, text)


def update_gmap(root, label):
    """Updates the map png included in the gui to allow the map to change as 
    the robot explores
    """
    map = Image.open("map.png")
    map = map.resize((500,400))
    map = ImageTk.PhotoImage(map)
    label.configure(image=map)
    label.image = map
    root.after(1000, lambda: update_gmap(root, label))


def cmd_entry(root, out, flags, robot_thread):
    """ Creates the entry field in the GUI into which a user may input GUI 
    commands, and binds it to the funciton which sets the flags shared by the 
    robot thread.
    """
    cmd_frame = tk.Frame(root, width=X_SIZE/2, height=150, bg="white")
    cmd_frame.pack
    cmd_frame.place(anchor='center', relx=.5, rely=.25)
    label = tk.Label(cmd_frame, text="Input Command Here", 
                     font="Times 20 bold underline")
    label.pack()
    entry = tk.Entry(cmd_frame, width=30, bg='light grey', font="Times 18 bold", 
                     justify='center')
    entry.pack()
    button = tk.Button(cmd_frame, text="Enter Comand", border=2, 
                       command=lambda: set_sigs(root, flags, entry, out, 
                                                   robot_thread))
    button.pack()


def resp_entry(root, responses, resp_flag):
    """ Creates the entry field which users may use to respond to any request 
    for data directly from the robot thread (i.e angle correction, filename, 
    etc.)
    """
    cmd_frame = tk.Frame(root, width=X_SIZE/2, height=150)
    cmd_frame.pack
    cmd_frame.place(anchor='center', relx=.175, rely=.85)
    label = tk.Label(cmd_frame, text="Respond to A Status Query Here", 
                     font="Times 18")
    label.pack()
    entry = tk.Entry(cmd_frame, width=30, bg='light grey', font="Times 15", 
                     justify='center')
    entry.pack()
    button = tk.Button(cmd_frame, text="Enter Response", 
                       command=lambda: send_resp(entry, responses, resp_flag), border=2)
    button.pack()


def send_resp(entry, responses, resp_flg):
    responses.append(entry.get)
    resp_flg[0] = True
    return


def on_close(root, robot_thread):
    """Defined the behavior which occurs when the gui is closed
    """
    kill_robot(robot_thread)
    root.destroy()


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


def cmp_input(entry, out):
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
    cmd = entry.get().lower()
    if cmd in CMD_DICT or cmd[:4] == 'goal' or cmd[:4]=='save':
        if cmd[:4] == 'goal' or cmd[:4]=='save':
            sigs = CMD_DICT[cmd[:4]]
        else:
            sigs = CMD_DICT[cmd]
        if cmd[:4] == "goal":
            goal = cmd[5:]
            sigs.append(goal)
        if cmd[:4] == "save":
            filen = cmd[5:]
            sigs.append(filen)
        else:
            sigs.append(None)
        return sigs
    else:
        post("Invalid command", out)


def create_rth(flags, outs, responses, resp_flag, map_num=None):
    """ Initializes and starts up the robot control thread
    """
    robot_thread = threading.Thread(name="RobotThread", \
                                 target=master,
                                 args=[flags, outs, responses, resp_flag, map_num])
    robot_thread.start()
    return robot_thread


def set_sigs(root, flags, entry, out, robot_thread):
    """ Sets the flags shared between the UI and the robot_thread to be
    responded to by the robot thread. 
    """
    temp = cmp_input(entry, out)
    if temp == None:
        return
    for i in range(len(flags)):
        if temp[i] != -1:
            flags[i] = temp[i]
        if flags[6] == 1:
                on_close(root, robot_thread)
