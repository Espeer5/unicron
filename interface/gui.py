"""This module contains the functions needed to initialize, run, and update the 
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
    
    #Initialize shared variables for thread communication
    messages = []
    flags = [False for _ in range(11)]
    responses = []
    resp_flag = [False]
    state = [(None, None), None]
    
    #Initialize Root Window
    root = tk.Tk()
    root.tk_setPalette(background="white")
    root.title("NormStorm Controller")
    root.configure(bg="white")
    root.geometry(f"{X_SIZE}x{Y_SIZE}")
    root.minsize(X_SIZE, Y_SIZE)
    root.maxsize(X_SIZE, Y_SIZE)

    #Insert the decorative heading bar into the root window
    top_frame = tk.Frame(root, width=X_SIZE, height=150, 
                         highlightbackground="black", highlightthickness=3)
    top_frame.pack()
    top_frame.place(anchor='center', relx=0.5, rely=.06)
    img = ImageTk.PhotoImage(Image.open(const.GRAPHX_PATH + 'ns_tbar.png'))
    img_label = tk.Label(top_frame, image=img)
    img_label.pack()

    #Insert the current map into the window
    map_frame = tk.Frame(root, width=X_SIZE/2)
    map_frame.pack()
    map_frame.place(anchor='center', relx=.75, y=450)
    map = Image.open(const.MAP_PATH)
    map = map.resize((500, 400))
    map = ImageTk.PhotoImage(map)
    map_label = tk.Label(map_frame, image=map)
    map_label.pack()

    #Create the output space for the bot to post messages to
    out = OSpace(root, messages)
    outs = (out[0], out[1], messages)
    post("WARNING: norman is feeling extra naughty (ง ͠° ͟ʖ ͡°)ง", outs)

    #Start the ROS worker thread.
    ros_thread = threading.Thread(name="ROSThread", 
                                  target=lambda: ros.runros(state, flags, outs))
    ros_thread.start()

    #Start the robot thread, and bind the command input box to control it
    robot_thread = threading.Thread(name="RobotThread", \
                                 target=master,
                                 args=[flags, outs, responses, resp_flag, 
                                       state])
    robot_thread.start()
    cmd_entry(root, outs, flags, robot_thread, ros_thread)
    resp_entry(root, responses, resp_flag)
    
    #Update the map in the interface at 1 hz
    root.after(1000, lambda: update_gmap(root, map_label))

    #Define what happens upon closing the GUI
    root.protocol("WM_DELETE_WINDOW", 
                  lambda: on_close(root, robot_thread, ros_thread))
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
    #Only updates when image not being written by robot thread
    try:
        #Load map.png and show it in the GUI
        map = Image.open(const.MAP_PATH)
        map = map.resize((500,400))
        map = ImageTk.PhotoImage(map)
        label.configure(image=map)
        label.image = map
        #Re-enqueue the map update function after 1 second to run at 1 hz
        root.after(1000, lambda: update_gmap(root, label))
    except Exception:
        #If file being written, do not load it, but continue updating at 1 hz
        root.after(1000, lambda: update_gmap(root, label))


def cmd_entry(root, out, flags, robot_thread, ros_thread):
    """ Creates the entry field in the GUI into which a user may input GUI 
    commands, and binds it to the functton which sets the flags shared by the 
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
                       command=lambda: send_resp(entry, responses, resp_flag),
                       border=2)
    button.pack()


def send_resp(entry, responses, resp_flg):
    """When a user inputs a message into the response entry, appends this 
    message to the shared messages array so that the thread requesting
    information can use the input data. Also uses response flag to mark new 
    message available.
    """
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
    """Upon the command from the UI, stops all activities of the ros thread, 
    shutting down the node and closing the ros thread.
    """
    ctypes.pythonapi.PyThreadState_SetAsyncExc(
    ctypes.c_long(rosthread.ident), ctypes.py_object(KeyboardInterrupt))
    rosthread.join()


def cmp_input(entry, out):
    """Accepts a user inputted command from the command entry box in the gui, 
    and sets the robot control flags array such that the bot behaves accordngly
    """
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
    if flags[6]:
        on_close(root, robot_thread, ros_thread)
