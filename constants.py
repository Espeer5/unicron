""" This file contains all constants relevant to controlling a robot
	for ME/EE/CS 129 """

# Authors: Edward Speer, Garrett Knuf
# Date: 4/16/23

# GPIO HARDWARE CONNECTIONS
L_MOTOR_PINS = (7, 8) # left motor pins (A, B)
R_MOTOR_PINS = (5, 6) # right motor pins (A, B)
IR_PINS = (14, 15, 18) # IR detector pins (left, middle, right)

#Motor PWM Control Constants

PWM_FREQ = 1000 # PWM frequency for motor control

#PWM Settings for each motor for each type of motion (l, r)
#Due to hardware inconsistencies, these had to be fine tuned
#and determined experimentally

STRAIGHT_S = (195, -225) #Speeds to drive straight

L_VEER_S = (159, -225) #Speeds to veer left

R_VEER_S = (205, -193) #Speeds to veer right

L_STEER_S = (148, -225) #Speeds to steer left

R_STEER_S = (205, -180) #Speeds to steer right

L_TURN_S = (92, -225) #Speeds to turn left

R_TURN_S = (205, -113) #Speeds to turn right

L_HOOK_S = (0, -225) #Speeds to hook left

R_HOOK_S = (205, 0) #Speeds to hook right

L_SPIN_S = (-165, -175) #Speeds to spin left

R_SPIN_S = (153, 183) #Speeds to spin right

#Detector Tau Values

INTER_T = .03 #Intersection Detector

LR_T = .01 #Left/Right Detector

NR_T = .009 #Next Road Detector

#Mapping Constants

#Relate heading to the change in position they cause
heading_map = {0:(0, 1), 1:(1, 1), 2:(1, 0), 3:(1, -1),
                4:(0, -1), 5:(-1, -1), 6:(-1, 0), 7:(-1, 1)}

#Map delta in position to the heading of the robot
invert_h_map= {(0, 1): 0, (1, 1): 1, (1, 0): 2, (1, -1):3,
                       (0, -1):4, (-1, -1): 5, (-1, 0): 6, (-1, 1): 7}

#Exploration conditions of each road in an Intersection
CONDITIONS = ["UNKNOWN", "UNDRIVEN", "NONE", "DRIVEN"]

#Aliases for individual exploration conditions

UNK = CONDITIONS[0]
UND = CONDITIONS[1]
NNE = CONDITIONS[2]
DRV = CONDITIONS[3]