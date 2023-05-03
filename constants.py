""" This file contains all constants relevant to controlling a robot
	for ME/EE/CS 129 """

# Authors: Edward Speer, Garrett Knuf
# Date: 4/16/23

# GPIO HARDWARE CONNECTIONS
L_MOTOR_PINS = (7, 8) # left motor pins (A, B)
R_MOTOR_PINS = (5, 6) # right motor pins (A, B)
IR_PINS = (14, 15, 18) # IR detector pins (left, middle, right)

# Due to hardware imbalance the base duty cycle for the motors is 
# offset to drive in a straight line
# 250
# 230
L_MOTOR_SPEED = 185 # The base PWM duty cycle for the left motor
R_MOTOR_SPEED = 225 # The base PWM duty cycle for the right motor

PWM_FREQ = 1000 # PWM frequency for motor control
