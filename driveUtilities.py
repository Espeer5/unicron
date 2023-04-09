# This file contains basic drive train functions without sensors
# Created for EE 129 Week 2 goals
# Delays will be used to control duration of function calls

# Define the motor pins.
LMOTOR_PINA = 7
LMOTOR_PINB = 8
RMOTOR_PINA = 5
RMOTOR_PINB = 6

# Other settings
PWM_FREQ = 1000

def driveSetup():
	# Set up the four pins as output
    io.set_mode(LMOTOR_PINA, pigpio.OUTPUT)
    io.set_mode(LMOTOR_PINB, pigpio.OUTPUT)
    io.set_mode(RMOTOR_PINA, pigpio.OUTPUT)
    io.set_mode(RMOTOR_PINB, pigpio.OUTPUT)
    
    # Set the PWM frequency
    io.set_PWM_frequency(LMOTOR_PINA, PWM_FREQ)
    io.set_PWM_frequency(LMOTOR_PINB, PWM_FREQ)
    io.set_PWM_frequency(RMOTOR_PINA, PWM_FREQ)
    io.set_PWM_frequency(RMOTOR_PINB, PWM_FREQ)

    # Clear all pins, just in case.
    io.set_PWM_dutycycle(LMOTOR_PINA, 0)
    io.set_PWM_dutycycle(LMOTOR_PINB, 0)
    io.set_PWM_dutycycle(RMOTOR_PINA, 0)
    io.set_PWM_dutycycle(RMOTOR_PINB, 0)

    print("Motors ready...")


# 
def accelerate(startSpeed, targetSpeed, duration):
	# Check for invalid arguments
	if targetSpeed < startSpeed:
		raise Exception("target speed cannot be smaller than start \
		speed when accelerating")
		
	if targetSpeed > 255:
		raise Exception("target speed cannot be greater than 255")
	
	# Increment speed over several intervals
	# precision is the number of steps to take while accelerating
	accelPrecision = 5
	for i in range(accelPrecision):
		pwm_level = startSpeed + ((targetSpeed - startSpeed) * ((i+1)/5))
		io.set_PWM_dutycycle(LMOTOR_PINA, pwm_level)
		io.set_PWM_dutycycle(0)
		io.set_PWM_dutycycle(RMOTOR_PINA, pwm_level)
		io.set_PWM_dutycycle(0)
		time.sleep(duration / 5)
	return targetSpeed
