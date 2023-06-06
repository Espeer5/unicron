"""This file contains the main loop which executes the robot code according to 
the user interface defined in ui.py. This demo loop simply executes the UI 
which accepts user input to determine the next behavior of the bot.
"""

# Authors: Edward Speer, Garrett Knuf
# Date: 6/3/23

import interface.gui as gui


if __name__ == "__main__":
    gui.run_gui()
    