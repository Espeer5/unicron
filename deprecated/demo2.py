""" This file contains the second weekly demo for CS/ME/EE 129 for team
    unicron. The demo demonstrates line following capabilities of the
    robot. There are two iterations of line following algorithms:
        (1) basic line following that stops when robot reaches end of line
	    where robot must start at least partially on line
	(2) robot goes back and forth along the line, turning around every
	    time it reaches the end; it can also start off of the line
	    and adjust accordingly
"""

# Authors: Edward Speer, Garrett Knuf
# Date: 4/16/23

from driveSystem import DriveSystem
from linesensor import LineSensor
import pigpio
import constants as const
import time
import traceback

controller_simple = {(0, 1, 0): ("STRAIGHT", None),
                     (1, 1, 1): ("STRAIGHT", None), \
                     (0, 1, 1): ("STEER", "RIGHT"), \
                     (0, 0, 1): ("STEER", "RIGHT"), \
		     (1, 1, 0): ("STEER", "LEFT"), \
		     (1, 0, 0): ("STEER", "LEFT")}
		
controller_complex = {(0, 1, 0): ("STRAIGHT", None),
                     (1, 1, 1): ("STRAIGHT", None), \
                     (0, 1, 1): ("TURN", "RIGHT"), \
                     (0, 0, 1): ("TURN", "RIGHT"), \
		     (1, 1, 0): ("TURN", "LEFT"), \
		     (1, 0, 0): ("TURN", "LEFT")}

def run_simple(driveSys, sensor):
    while True:
        reading = sensor.read()
        if reading not in controller_simple:
            break
        driveSys.drive(controller_simple.get(reading)[0],
	controller_simple.get(reading)[1])
        # time.sleep(0.2) starts to wobble
	# time.sleep(0.5) wobbles off the tape


def run_complex(driveSys, sensor):
    lastReading = None
    while True:
        reading = sensor.read()
	# robot is entirely off the line
        if reading == (0, 0, 0):
	    # very beginning of code (robot has not found line yet)
            if lastReading == None:
                driveSys.drive("STRAIGHT")
	    # robot has reached end of line and needs to turn around
            elif lastReading == (0, 1, 0) or lastReading == (0, 1, 1) \
	    or lastReading == (1, 1, 0):
                # drive a little bit past the line to make turn around easier
                driveSys.drive("STRAIGHT")
                time.sleep(0.2)
		# spin around and set state such that the robot is searching
		# for the left edge of the line
                driveSys.drive("SPIN", "LEFT")
                lastReading = (1, 0, 0)
                time.sleep(0.85)
	    # continue following line 
            else:
                driveSys.drive(controller_complex.get(lastReading)[0], \
		controller_complex.get(lastReading)[1])
        # robot is at least partially on line so continue to drive straight 
        elif reading in controller_complex: 
            driveSys.drive(controller_complex.get(reading)[0], \
	    controller_complex.get(reading)[1])
            lastReading = reading
	# unknown state of controller so raise exception
        else:
            raise BaseException("Unknown state of IR controller readings")
        

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
	# run_simple(driveSys, IRSense)
        run_complex(driveSys, IRSense)
    except BaseException as ex:		
        # Report the error then continue with the normal shutdown
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Turn off hardware then disconnect the interface
    print("Turning off...")
    driveSys.stop()
    io.stop()
    
