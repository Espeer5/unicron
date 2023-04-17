"""
This file implements an IR object class. An instance of this class reads in data from one
infrared LED sensor used in ME/CS/EE 129.

Authors: Edward Speer, Garrett Knuf
Date: 4/16/2023
"""

import pigpio

class IR():
    """ An object oriented interface for reading data from an infrared LED sensor.
    This sensor reads low when detecting a white surface, high when detecting a
    black surface underneath. 

    Arguments:
        io: A pigpio io object
        pin: signal pin of IR sensor
    """

    def __init__(self, io, pin):
        self.io = io
        self.pin = pin
        io.set_mode(pin, pigpio.INPUT)

    def read(self):
        """ Return the current reading of the IR sensor associated with this IR
        instance. """
        return self.io.read(self.pin)
        
