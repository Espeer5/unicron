""" This file contains the third weekly demo for CS/ME/EE 129 for team Unicron. This file causes 
    the robot to exhibit line following behavior using filtering to distinguish deviations from 
    the tape line from sensor noise. The robot uses detectors to determine the appropriate action 
    to take when it has driven off the line, and to stop upon reaching a valid intersection.
    This demo can be adapted to perform any behavior of the bot given in the behaviors module.
"""

# Authors: Edward Speer, Garrett Knuf
# Date: 4/16/23

from driveSystem import DriveSystem
from linesensor import LineSensor
import pigpio
import constants as const
import traceback
from behaviors import alternate

if __name__ == "__main__":

    # Prepare the GPIO interface
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
	    alternate(driveSys, IRSense)
    except BaseException as ex:		
        # Report the error then continue with the normal shutdown
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Turn off hardware then disconnect the interface
    print("Turning off...")
    driveSys.stop()
    io.stop()

