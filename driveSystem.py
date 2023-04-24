"""
This module instantiates a DriveSystem class for the robot build in 
ME/CS/EE 129. This class instantiates a left/right pair of Motor
objects and implements a discrete set of drive instructions for 
a robot whose wheels are powered by those motors.

Authors: Edward Speer, Garrett Knuf
Date: 4/16/2023
"""

# Imports
from motor import Motor
import pigpio
import constants as const
import time

class DriveSystem():
    """ This class instantiates the left and right Motor objects and provides 
    basic control functionality to use these objects to drive and steer the bot
    based on feedback from the LineSensor data 

    Arguments:
        io: pigpio GPIO interface object
        leftMPins: left motor pins tuple
        rightMPins: right motor pins tuple
        PWMFreq: Pulse Width modulation frequency for motor speed control
    """

    # Available drive modes and their corresponding offsets
    # Define Motor speed offsets in as fractions of normal motor speed
    MODES = {"STRAIGHT": 0, "VEER": 1/7, "STEER": 1/4.9,
            "TURN": 1/2, "HOOK": 1, "SPIN": 2}

    def __init__(self, io, leftMPins, rightMPins, PWMFreq):
        # instantiate the left and right Motor instances
        self.left_motor = Motor(io, leftMPins, PWMFreq)
        self.right_motor = Motor(io, rightMPins, PWMFreq)
        print("Motors Ready...")
       
    def stop(self):
        """ Immediately stops all motion of the robot controlled by the DriveSystem 
        object. """
        self.left_motor.disable()
        self.right_motor.disable()

    def drive(self, mode, direction=None):
        """ Engages the robot motors to drive the robot along a certain trajectory

        Arguments:
            mode: One of "STRAIGHT", "VEER", "STEER", "TURN", "HOOK" or "SPIN"
                  indicating the extent of curvature in the path of the bot
            direction (optional): Either "LEFT" or "RIGHT" indicating the direction of 
                       curvature. Not required when mode is "STRAIGHT"

        Raises:
            Exception if direction not specified for a turning trajectory, or if 
            mode specified is invalid, or if direction specified is not valid
        """
        # Ensure a valid mode is given
        if(mode not in self.MODES):
            raise Exception("DriveSystem.drive: invalid mode specified")

        #Ensure a valid direction is specified for a curving trajectory
        if(direction not in ["LEFT", "RIGHT"] and direction != None):
            raise Exception("DriveSystem.drive: invalid direction specified")
        if(direction == None and mode != "STRAIGHT"):
            raise Exception("DriveSystem.drive: Must specify LEFT or RIGHT")
        
        # If direction is left, slow the left motor
        if(direction == "LEFT"):
            self.left_motor.setSpeed(const.L_MOTOR_SPEED - (self.MODES[mode]*const.L_MOTOR_SPEED))
            self.right_motor.setSpeed(-const.R_MOTOR_SPEED)

        # If direction is right, slow the right motor
        # This case also runs if direction is straight with offset 0
        else:
            self.left_motor.setSpeed(const.L_MOTOR_SPEED)
            self.right_motor.setSpeed(-(const.R_MOTOR_SPEED - (self.MODES[mode]*const.R_MOTOR_SPEED)))


def test_flower():
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio unicron!")
        sys.exit(0)
    print("GPIO ready...")
    driveSys = DriveSystem(io, const.L_MOTOR_PINS, const.R_MOTOR_PINS, \
                           const.PWM_FREQ)
    for style in driveSys.MODES:
        if style in ["STRAIGHT", "VEER", "STEER"]:
            continue
        for direction in ["LEFT", "RIGHT"]:
            driveSys.drive(style, direction)
            time.sleep(4)
            driveSys.stop()
            input("Hit Return")
        
