"""This file is the main executable for the robot in ME/CS/EE 129 Spring '23, 
which simply starts up the GUI for the robot, and in turn the robot along with 
ROS node are started.

Authors: Edward Speer, Garrett Knuf
Date: 6/3/23
"""

import interface.gui as gui


if __name__ == "__main__":
    #Start up GUI along with all control threads
    gui.run_gui()
