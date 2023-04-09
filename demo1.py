# This file contains the first weekly demo for CS/ME/EE 129 for team unicron
# The demo drives the robot in a square, executing a point turn at each corner
# This file requires the utility functions in driveUtilities.py

# Authors: Edward Speer, Garrett Knuf
# Date: 4/9/23

#imports
import time

TOP_SPEED = #TODO
ACCEL_DURATION = #TODO
DRIVE_DUR = #TODO

if __name__ == "__main__":
    
    # Create a state variable for the current linear speed of the robot
    current_speed = 0

    # Prepare GPIO Interface, prepare PWM, and clear all pins
    driveSetup()

    #For each of the 4 sides of the square, drive forward, then rotate by 90 degrees
    for i in range(4):

        # Accelerate up to speed in the forward direction
        current_speed = accelerate(current_speed, TOP_SPEED, ACCEL_DURATION)
        
        # Drive forward for the drive duration
        time.sleep(DRIVE_DUR)

        # Deccelerate down to a stop
        current_speed = deccelerate(current_speed, )

        # Execute a 90 degree clockwise turn

