""" This file contains the first weekly demo for CS/ME/EE 129 for team unicron
    The demo drives the robot in a square, executing a point turn at each corner 
    This file requires the utility functions in driveUtilities.py as well as 
    the motor object defined in motor.py """

# Authors: Edward Speer, Garrett Knuf
# Date: 4/9/23

#imports
import driveUtilities as du
import pigpio
from motor import Motor

TOP_SPEED = 235
DRIVE_DUR = 1
ROT_SPEED = 200
ROT_DUR = .65

if __name__ == "__main__":

    # Prepare the GPIO interface
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    print("GPIO ready...")

    # Create a Motor class instance for each motor
    left_motor = Motor(io, du.L_MOTOR_PINA, du.L_MOTOR_PINB, du.PWM_FREQ) 
    right_motor = Motor(io, du.R_MOTOR_PINA, du.R_MOTOR_PINB, du.PWM_FREQ)

    #For each of the 4 sides of the square, drive forward, then rotate by 90 degrees
    for i in range(4):

        # Drive in the direction the robot is facing for the given duration
        du.driveForward(left_motor, right_motor, TOP_SPEED, DRIVE_DUR)

        # Execute a 90 degree clockwise turn
        du.turnRight(left_motor, right_motor, ROT_SPEED, ROT_DUR)

    # Disable the motors and disconnect the interface
    du.disableDrive(left_motor, right_motor)
    io.stop()
