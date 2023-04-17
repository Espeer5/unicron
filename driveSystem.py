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
from constants import L_MOTOR_SPEED, R_MOTOR_SPEED

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
    # Define Motor speed offsets in terms of deviation from the average l/r speed
    MOTOR_SPEED = (L_MOTOR_SPEED + R_MOTOR_SPEED) / 2
    MODES = {"STRAIGHT": 0, "VEER": MOTOR_SPEED/12, "STEER": MOTOR_SPEED/6,
            "TURN": MOTOR_SPEED/2, "HOOK": MOTOR_SPEED, "SPIN": 2 * MOTOR_SPEED}

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
            self.left_motor.setSpeed(L_MOTOR_SPEED - self.MODES.get(mode))
            self.right_motor.setSpeed(-R_MOTOR_SPEED)

        # If direction is right, slow the right motor
        # This case also runs if direction is straight with offset 0
        else:
            self.left_motor.setSpeed(L_MOTOR_SPEED)
            self.right_motor.setSpeed(-(R_MOTOR_SPEED - self.MODES.get(mode)))
