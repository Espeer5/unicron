""" This file contains the goals #7 demo for CS/ME/EE 129 for team Unicron.
"""

# Authors: Edward Speer, Garrett Knuf
# Date: 5/15/23

from driving.driveSystem import DriveSystem
from sensing.proximitysensor import ProximitySensor
import pigpio
import constants as const
import traceback
from behaviors import herd, wall_follow, discrete_wall_follow, proportional_wall_follow
from textwrap import dedent

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
    ultraSense = ProximitySensor(io)

    try:
        cmd = input(dedent("""\
                     1: Herd the Normstorm
                     2: Discrete wall-follow
                     3: Proportional feedback wall-follow\n"""))
        if cmd == "1":
            herd(driveSys, ultraSense)
        elif cmd == "2":
            direction = input("Follow a 'L' or 'R' wall: ").upper()
            dirs = {'L': "LEFT", 'R': "RIGHT"}
            wall_follow(driveSys, ultraSense, dirs[direction],
                        discrete_wall_follow)
        elif cmd == "3":
            direction = input("Follow a 'L' or 'R' wall: ").upper()
            dirs = {'L': "LEFT", 'R': "RIGHT"}
            wall_follow(driveSys, ultraSense, dirs[direction],
                        proportional_wall_follow)      
        else:
            print("Invalid command, exiting...")
    except BaseException as ex:		
        # Report the error then continue with the normal shutdown
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Turn off hardware then disconnect the interface
    print("Turning off...")
    ultraSense.shutdown()
    driveSys.stop()
    io.stop()

