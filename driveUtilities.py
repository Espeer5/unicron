""" This file contains basic drive train functions without sensors 
    Delays will be used to control duration of function calls"""

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

def driveForward(left_motor, right_motor, speed, duration):
    left_motor.setSpeed(speed)
    right_motor.setSpeed(-(speed+20))
    time.sleep(duration)
    
def driveBackward(left_motor, right_motor, speed, duration):
    left_motor.setSpeed(-speed)
    right_motor.setSpeed(speed)
    time.sleep(duration)
    
def turnLeft(left_motor, right_motor, speed, duration):
    left_motor.setSpeed(-speed)
    right_motor.setSpeed(-speed)
    time.sleep(duration)
    
def turnRight(left_motor, right_motor, speed, duration):
    left_motor.setSpeed(speed)
    right_motor.setSpeed(speed)
    time.sleep(duration)
    
def disableDrive(left_motor, right_motor):
    left_motor.disable()
    right_motor.disable()
