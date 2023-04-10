""" This file is the class for basic motor control
    It can initialize, disable, and control a basic DC motor"""

# Authors: Edward Speer, Garrett Knuf
# Date: 4/9/23

import pigpio

class Motor():
	
	def __init__ (self, io, legAPin, legBPin, pwm_freq):
		"""Initialize motor
		Sets the pins as output, the PWM frequency, and sets speed to 0
		
		Args:
			io: gpio object
			legAPin: pin1
			legBPin: pin2
			pwm_freq: PWM frequency (1000 Hz recommended)
		"""
		self.io = io
		self.legAPin = legAPin
		self.legBPin = legBPin
		self.pwm_freq = pwm_freq
		
		# Set up the four pins as output
		self.io.set_mode(legAPin, pigpio.OUTPUT)
		self.io.set_mode(legBPin, pigpio.OUTPUT)
		
		# Set resolution for PWM
		io.set_PWM_range(legAPin, 255)
		io.set_PWM_range(legBPin, 255)
		
		# Set the PWM frequency
		self.io.set_PWM_frequency(legAPin, pwm_freq)
		self.io.set_PWM_frequency(legBPin, pwm_freq)

		# Clear all pins, just in case.
		self.io.set_PWM_dutycycle(legAPin, 0)
		self.io.set_PWM_dutycycle(legBPin, 0)
		
	def setPWMFreq (self, freq):
		"""Sets the PWM frequency of the motor
		
		Args:
			freq: frequency (1000 Hz recommended)
		"""
		self.io.set_PWM_frequency(self.legAPin, pwm_freq)
		self.io.set_PWM_frequency(self.legBPin, pwm_freq)
		
	def setSpeed(self, speed):
		"""Sets the speed of the motor
		A positive speed spins the motor forward
		A negative speed spins the motor reverse
		
		Args:
			speed: speed of motor -255 <= speed <= 255
		"""
		if (speed >= 0):
			self.io.set_PWM_dutycycle(self.legAPin, speed)
			self.io.set_PWM_dutycycle(self.legBPin, 0)
		else:
			self.io.set_PWM_dutycycle(self.legAPin, 0)
			self.io.set_PWM_dutycycle(self.legBPin, -speed)
			
	def disable(self):
		"""Disables motor in case of shutdown"""
		self.io.set_PWM_dutycycle(self.legAPin, 0)
		self.io.set_PWM_dutycycle(self.legBPin, 0)
