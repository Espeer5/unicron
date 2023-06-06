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
import ui

#GUI Window size
X_SIZE = 950
Y_SIZE = 800

#Maximum number of posted messages saved in the GUI at a time
MESSAGE_MEM = 18

def create_root():
    """ Initializes the GUI window and populates it with the features making 
    up the control interface. Executes the main loop of the graphics window
    (and hence doesn't return)
    """
    messages = []

    root = tk.Tk()
    root.title("NormStorm Controller")
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
    map_frame.place(anchor='center', relx=.65, y=500)

    map = ImageTk.PhotoImage(Image.open("map.png"))

    map_label = tk.Label(map_frame, image=map)
    map_label.pack()

    out = OSpace(root, messages)
    outs = (out[0], out[1], messages)
    cmd_entry(root, outs)
    resp_entry(root, outs)

    post("WARNING: norman is feeling\nextra naughty (ง ͠° ͟ʖ ͡°)ง", outs)
    
    root.after(1000, lambda: update_gmap(root, map_label))

    root.mainloop()


def OSpace(root, messages):
    """ Creates the output space for message posting by the robot to the GUI.
    This function returns the output object shared between threads which allows 
    message posting to the GUI.
    """
    OFrame = tk.Frame(root, width=X_SIZE/4, height=400)
    OFrame.pack()
    OFrame.place(anchor='center', relx=.15, y=480)
    OCanvas = tk.Canvas(OFrame, width=X_SIZE/4, height=400)
    OCanvas.create_text(100, 30, text="Status Messages", fill="black", 
                        font="Times 20 bold underline")
    text = OCanvas.create_text(100, 50, text=get_messages(messages), 
                               fill="black", font="Times 15", anchor="n")
    OCanvas.pack()
    return (OCanvas, text)


def post(message, out):
    """Posts the given message to the GUI at the location indicated by the out 
    array
    """
    out[2].append(message)
    out[0].itemconfig(out[1], text=get_messages(out[2]))


def update_gmap(root, label):
    """Updates the map png included in the gui to allow the map to change as 
    the robot explores
    """
    map = ImageTk.PhotoImage(Image.open("map.png"))
    label.configure(image=map)
    label.image = map
    root.after(1000, lambda: update_gmap(root, label))


def get_messages(messages):
    """ Formats the messages array into a string which may then be displayed in
    the ouput space of the GUI.
    """
    if len(messages) > MESSAGE_MEM:
        messages = messages[-MESSAGE_MEM:]
    str = ""
    for mess in messages:
        str += f"{mess}\n"
    return str


def cmd_entry(root, out):
    """ Creates the entry field in the GUI into which a user may input GUI 
    commands, and binds it to the funciton which sets the flags shared by the 
    robot thread.
    """
    cmd_frame = tk.Frame(root, width=X_SIZE/2, height=150)
    cmd_frame.pack
    cmd_frame.place(anchor='center', relx=.5, rely=.25)
    label = tk.Label(cmd_frame, text="Input Command Here", 
                     font="Times 20 bold underline")
    label.pack()
    entry = tk.Entry(cmd_frame, width=30, bg='light grey', font="Times 18 bold", 
                     justify='center')
    entry.pack()
    button = tk.Button(cmd_frame, text="Enter Comand", border=2, 
                       command=lambda: ui.cmp_input(entry, out))
    button.pack()


def resp_entry(root, out):
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
    button = tk.Button(cmd_frame, text="Enter Response", border=2)
    button.pack()


if __name__ == '__main__':
    create_root()
