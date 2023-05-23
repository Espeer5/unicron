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

    def __init__(self, io, leftMPins, rightMPins, PWMFreq):
        # instantiate the left and right Motor instances
        self.left_motor = Motor(io, leftMPins, PWMFreq)
        self.right_motor = Motor(io, rightMPins, PWMFreq)
       
    def stop(self):
        """ Immediately stops all motion of the robot controlled by the DriveSystem 
        object. """
        self.left_motor.disable()
        self.right_motor.disable()

    def pwm(self, PWM_L, PWM_R):
        """A simple utility which sets the driveSystem motors to the passed in l and r 
        PWM values.
        """
        self.left_motor.setSpeed(PWM_L)
        self.right_motor.setSpeed(PWM_R)

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
        if(mode not in const.MODES):
            raise Exception("DriveSystem.drive: invalid mode specified")

        #Ensure a valid direction is specified for a curving trajectory
        if direction not in ["LEFT", "RIGHT"] and direction != None:
            raise Exception("DriveSystem.drive: invalid direction specified")
        if direction == None and mode not in ["STRAIGHT", "BACKWARDS"]:
            raise Exception("DriveSystem.drive: Must specify LEFT or RIGHT")
        
        #Set the left and right motor speeds based on mode and direction
        self.pwm(const.MODES[mode][direction][0], const.MODES[mode][direction][1])
    
    def kick(self, direction):
        """Executes a very short timed full power spin in the specified direction in order 
        to gain a small amount of momentum and overcome static friction in initiating a turn. 
        This enables the turning of the bot to be slightly more consisten and therefore allows 
        angle measurement to be slightly more accurate.
        """
        direc = 1
        if direction == "LEFT":
            direc = -1
        self.pwm(direc * 255, direc * 255)
        time.sleep(const.KICK_TIME)
        self.stop()


def test_flower():
    """Execute a test pattern demonstrating all of the movements of the DriveSystem
    """
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio unicron!")
        sys.exit(0)
    print("GPIO ready...")
    driveSys = DriveSystem(io, const.L_MOTOR_PINS, const.R_MOTOR_PINS, \
                           const.PWM_FREQ)
    for style in const.MODES:
        #if style not in ["SPIN"]:
            #continue
        for direction in ["LEFT", "RIGHT"]:
            input("Hit Return")
            driveSys.drive(style, direction)
            time.sleep(4)
            driveSys.stop()
                   
