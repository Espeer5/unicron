"""This module contains the functions needed to initialize and update the 
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
import constants as const
import ros

#GUI Window size
X_SIZE = 950
Y_SIZE = 680

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

    post("WARNING: norman is feeling extra naughty (ง ͠° ͟ʖ ͡°)ง", outs)
    state = [(None, None), None]

    # Start the ROS worker thread.
    ros_thread = threading.Thread(name="ROSThread", target=lambda: ros.runros(state, flags))
    ros_thread.start()

    robot_thread = create_rth(flags, outs, responses, resp_flag, state)

    cmd_entry(root, outs, flags, robot_thread, ros_thread)
    resp_entry(root, responses, resp_flag)
    
    root.after(1000, lambda: update_gmap(root, map_label))
    root.protocol("WM_DELETE_WINDOW", lambda: on_close(root, robot_thread, ros_thread))
    try:
        root.mainloop()
    except KeyboardInterrupt:
        on_close(root, robot_thread, ros_thread)


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
    #Try-except protects from race condition in updating map png file
    try:
        map = Image.open("map.png")
        map = map.resize((500,400))
        map = ImageTk.PhotoImage(map)
        label.configure(image=map)
        label.image = map
        root.after(1000, lambda: update_gmap(root, label))
    except Exception:
        root.after(1000, lambda: update_gmap(root, label))


def cmd_entry(root, out, flags, robot_thread, ros_thread):
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
                                                   robot_thread, ros_thread))
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
    responses.append(entry.get())
    resp_flg[0] = True
    return


def on_close(root, robot_thread, ros_thread):
    """Defines the behavior which occurs when the gui is closed
    """
    kill_robot(robot_thread)
    kill_ros(ros_thread)
    root.quit()


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

def kill_ros(rosthread):
    ctypes.pythonapi.PyThreadState_SetAsyncExc(
    ctypes.c_long(rosthread.ident), ctypes.py_object(KeyboardInterrupt))
    rosthread.join()


def cmp_input(entry, out):
    cmd = entry.get().lower()
    if cmd in const.CMD_DICT or cmd[:4] == 'goal' or cmd[:4]=='save':
        if cmd[:4] == 'goal' or cmd[:4]=='save':
            sigs = const.CMD_DICT[cmd[:4]]
        else:
            sigs = const.CMD_DICT[cmd]
        sigs.append(None)
        if cmd[:4] == "goal":
            goal = cmd[5:]
            sigs[const.DATA] = goal
        elif cmd[:4] == "save":
            filen = cmd[5:]
            sigs[const.DATA] = filen
        return sigs
    else:
        post("Invalid command", out)


def create_rth(flags, outs, responses, resp_flag, state, map_num=None):
    """ Initializes and starts up the robot control thread
    """
    robot_thread = threading.Thread(name="RobotThread", \
                                 target=master,
                                 args=[flags, outs, responses, resp_flag, state, map_num])
    robot_thread.start()
    return robot_thread


def set_sigs(root, flags, entry, out, robot_thread, ros_thread):
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
                on_close(root, robot_thread, ros_thread)
