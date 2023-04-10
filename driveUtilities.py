""" This file contains basic drive train functions without sensors 
    Delays will be used to control duration of function calls """

# Authors: Edward Speer, Garrett Knuf
# Date: 4/9/23

# Imports
from motor import *
import time

# Define the motor pins.
L_MOTOR_PINA = 7
L_MOTOR_PINB = 8
R_MOTOR_PINA = 5
R_MOTOR_PINB = 6

# Motor control settings
PWM_FREQ = 1000


def driveStraight(left_motor, right_motor, speed, duration):
    """
      Drives the robot in a straight line at the given speed for the given duration
    """
    left_motor.setSpeed(speed)
    right_motor.setSpeed(-(speed+18))
    time.sleep(duration)
    

def rotate(left_motor, right_motor, speed, duration):
    """
      Rotates the robot in place by performing a point turn
    """
    left_motor.setSpeed(speed)
    right_motor.setSpeed(speed)
    time.sleep(duration)
    

def disableDrive(left_motor, right_motor):
    """
      Immediately stops all motor activity for the robot
    """
    left_motor.disable()
    right_motor.disable()
