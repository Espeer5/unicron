"""
This module instantiates a DriveSystem class for the robot build in 
ME/CS/EE 129. This class instantiates a left/right pair of Motor
objects and implements a discrete set of drive instructions for 
a robot whose wheels are powered by those motors.

Authors: Edward Speer, Garrett Knuf
Date: 4/16/2023
"""

# Imports
from driving.drivers.motor import Motor
import pigpio
import constants as const
import time
import sys

sys.path.insert(0, '/home/robot/project')

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
    MODES = {"STRAIGHT": {None: const.STRAIGHT_S, "LEFT": const.STRAIGHT_S,
        "RIGHT": const.STRAIGHT_S},
            "VEER": {"LEFT": const.L_VEER_S, "RIGHT": const.R_VEER_S},
            "STEER": {"LEFT": const.L_STEER_S, "RIGHT": const.R_STEER_S},
            "TURN": {"LEFT": const.L_TURN_S, "RIGHT": const.R_TURN_S},
            "HOOK": {"LEFT": const.L_HOOK_S, "RIGHT": const.R_HOOK_S},
            "SPIN": {"LEFT": const.L_SPIN_S, "RIGHT": const.R_SPIN_S}}

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
        
        #Set the left and right motor speeds based on mode and direction
        self.left_motor.setSpeed(self.MODES[mode][direction][0])
        self.right_motor.setSpeed(self.MODES[mode][direction][1])
    
    def kick(self, direction):
        direc = 1
        if direction == "LEFT":
            direc = -1
        self.left_motor.setSpeed(direc * 255)
        self.right_motor.setSpeed(direc * 255)
        time.sleep(.07)
        self.stop()

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
        if style not in ["SPIN"]:
            continue
        for direction in ["LEFT", "RIGHT"]:
            input("Hit Return")
            driveSys.drive(style, direction)
            time.sleep(4)
            driveSys.stop()
                   
