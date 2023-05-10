""" This file contains the fifth weekly demo for CS/ME/EE 129 for team Unicron.
"""

# Authors: Edward Speer, Garrett Knuf
# Date: 5/7/23

from driving.driveSystem import DriveSystem
from sensing.linesensor import LineSensor
import pigpio
import constants as const
import traceback
from behaviors import auto_djik, manual_djik, navigate
import time

if __name__ == "__main__":

    # Prepare tshe GPIO interface
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio unicron!")
        sys.exit(0)
    print("GPIO ready...")
    
    # Create objects
    driveSys = DriveSystem(io, const.L_MOTOR_PINS, const.R_MOTOR_PINS, \
                           const.PWM_FREQ)
    IRSense = LineSensor(io, const.IR_PINS)

    try:
        cmd = input("1: Explore the map manually\n" + 
                    "2: Navigate a map using Djikstra's\n" +
                    "3: Auto-explore the map\n")
        if cmd == "1":
            navigate(driveSys, IRSense)
        elif cmd == "2":
            manual_djik(driveSys, IRSense)
        elif cmd == "3":
            auto_djik(driveSys, IRSense)
        else:
            print("Invalid command, exiting...")
    except BaseException as ex:		
        # Report the error then continue with the normal shutdown
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Turn off hardware then disconnect the interface
    print("Turning off...")
    driveSys.stop()
    io.stop()

