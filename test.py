"""This module allows the user to specify on the command line the 
hardware test they they want to run on the robot, and then runs the 
corresponding test for the purposes of checking if the hardware is functioning 
as it should be.

Authors: Edward Speer, Garrett Knuf
Date: 6/10/23
"""

import sys
from sensing.proximitysensor import test
from driving.driveSystem import test_flower
from sensing.linesensor import test_ls

if __name__ == "__main__":
    #Based on the argument passed on the command line, test the hardware
    mode = sys.argv[1]
    print(mode)
    if mode == 'Motors':
        test_flower()
    if mode == 'IRSensors':
        test_ls()
    if mode == 'Ultrasounds':
        test()
    else:
        print("Invalid hardware specified")