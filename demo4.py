""" This file contains the fourth weekly demo for CS/ME/EE 129 for team Unicron.
"""

# Authors: Edward Speer, Garrett Knuf
# Date: 4/30/23

from driving.driveSystem import DriveSystem
from sensing.linesensor import LineSensor
import pigpio
import constants as const
import traceback
from behaviors import navigate, auto_explore

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
        demo = input("1 for manual navigation and 2 for autonomous mode: ")
        if demo == "1":
            navigate(driveSys, IRSense)
        elif demo == "2":
            auto_explore(driveSys, IRSense)
    except BaseException as ex:		
        # Report the error then continue with the normal shutdown
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Turn off hardware then disconnect the interface
    print("Turning off...")
    driveSys.stop()
    io.stop()

