"""
This file implements an Ultrasound Sensor. An instance of this class read in
data from one ultrasound sensor used in ME/CS/EE 129

Authors: Edward Speer, Garrett Knuf
Date: 5/13/2023
"""

import pigpio
from time import sleep

class Ultrasound():
    """ An object oriented interface for reading and calculating data from an
    ultrasound sensor. Utilizes callback functions to detect rising/falling
    edges of echo pin of ultrasound sensor. The read() function will return
    the most recent calculated distance measured from the sensor.
    
    Arguments:
        io: A pigpio io object
        pintrig: trigger pin of ultrasonic sensor
        pinecho: echo pin of ultrasonic sensor
    """

    SPEED_OF_SOUND = 343000000 # meters per microsecond

    def __init(self, io, pintrig, pinecho):
        self.io = io
        io.set_mode(pintrig, pigpio.OUTPUT)
        io.set_mode(pinecho, pigpio.INPUT)
        cbrise = io.callback(pinecho, pigpio.RISING_EDGE, self.rising)
        cbfall = io.callback(pinecho, pigpio.FALLING_EDGE, self.falling)
        self.risetick = 0
        self.last_dist = 0

    def trigger(self):
        """ pulls the trigger pin high for 10 microseconds """
        io.write(self.pintrig, 1)
        time.sleep(0.000010)
        io.write(self.pinecho, 0)

    def rising(self, pin, level, ticks):
        """ stores time of last rising edge of echo pin """
        self.risetick = ticks

    def falling(self, pin, level, ticks):
        """ calculates and stores distance travelled """
        dt = ticks - self.rise
        # prevent integer overflow
        if dt < 0:
            dt += 2 ** 32
        print("Flight time: " + str(dt))
        self.last_dist = dt * SPEED_OF_SOUND / 2

    def read(self):
        """ returns distance of last ultrasound reading """
        return self.last_dist

