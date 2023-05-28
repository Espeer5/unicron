""" This file contains all constants relevant to controlling a robot
	for ME/EE/CS 129 """

# Authors: Edward Speer, Garrett Knuf
# Date: 4/16/23

#Adjust for the battery charge
BATT_LIFE = .91

# GPIO HARDWARE CONNECTIONS
L_MOTOR_PINS = (7, 8) # left motor pins (A, B)
R_MOTOR_PINS = (5, 6) # right motor pins (A, B)
IR_PINS = (14, 15, 18) # IR detector pins (left, middle, right)
L_ULTRASOUND_PINS = (13, 16) # left ultrasound pins (trigger, echo)
C_ULTRASOUND_PINS = (19, 20) # center ultrasound pins (trigger, echo)
R_ULTRASOUND_PINS = (26, 21) # right ultrasound pins (trigger, echo)

#Motor PWM Control Constants

PWM_FREQ = 1000 # PWM frequency for motor control

#PWM Settings for each motor for each type of motion (l, r)
#Due to hardware inconsistencies, these had to be fine tuned
#and determined experimentally
MODES = {"STRAIGHT": {None: (200, -225), "LEFT": (200, -225),
        "RIGHT": (200, -225)},
            "VEER": {"LEFT": (159, -225), "RIGHT": (205, -193)},
            "STEER": {"LEFT": (148, -225), "RIGHT": (205, -180)},
            "TURN": {"LEFT": (92, -225), "RIGHT": (205, -113)},
            "HOOK": {"LEFT": (0, -215), "RIGHT": (190, 0)},
            "SPIN": {"LEFT": (-180, -190), "RIGHT": (158, 188)},
            "BACKWARDS": {None: (-195, 225)}}

#Detector Tau Values

INTER_T = .038 #Intersection Detector

LR_T = .002 #Left/Right Detector

NR_T = .017 #Next Road Detector

#Driving Constants

# The feedback law governing steady state line following
FEEDBACK_TABLE = {(0, 1, 0): ("STRAIGHT", None),
                     (1, 1, 1): ("STRAIGHT", None), \
                     (0, 1, 1): ("TURN", "RIGHT"), \
                     (0, 0, 1): ("TURN", "RIGHT"), \
		             (1, 1, 0): ("TURN", "LEFT"), \
                     (1, 0, 0): ("TURN", "LEFT")}

#Map directions from user input to string and integer directions
dirMap = {"L":("LEFT", 1), "R": ("RIGHT", -1), "S": "STRAIGHT"}

# An alias for the intersection found exit condition from line follow
SUCCESS = 1
FAILURE = 2
OBJECT_COLLISION_DIST = 0.1

#How long to pull up for when arriving to an intersection
PULLUP_T = .475

#The amount of time over which a full power kick is executed initiating a turn
KICK_TIME = .07

#Mapping Constants

#Relate heading to the change in position they cause
heading_map = {0:(0, 1), 1:(1, 1), 2:(1, 0), 3:(1, -1),
                4:(0, -1), 5:(-1, -1), 6:(-1, 0), 7:(-1, 1)}

#Map delta in position to the heading of the robot
invert_h_map= {(0, 1): 0, (1, 1): 1, (1, 0): 2, (1, -1):3,
                       (0, -1):4, (-1, -1): 5, (-1, 0): 6, (-1, 1): 7}

#Exploration conditions of each road in an Intersection
CONDITIONS = ["UNKNOWN", "UNDRIVEN", "NONE", "DRIVEN"]
STREET_CONDITIONS = ["BLOCKED", "UNBLOCKED"]

#Aliases for individual exploration conditions
UNK = CONDITIONS[0]
UND = CONDITIONS[1]
NNE = CONDITIONS[2]
DRV = CONDITIONS[3]
BLK = STREET_CONDITIONS[0]
UNB = STREET_CONDITIONS[1]

# Ultrasonic and wall-following constants
WALL_FOLLOW_DIST = 0.3
US_DELAY = 0.05
US_THRESHOLD = 0.1
WALL_FOLLOW_PROP = 700

