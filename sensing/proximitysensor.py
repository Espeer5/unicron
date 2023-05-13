"""
This file implements a ProximitySensor object which contains 3 Ultrasound
sensor. It provide functionality to trigger and read each of the sensors.

Authors: Edward Speer, Garrett Knuf
Date: 5/13/2023
"""

from sensing.drivers.ultrasound import Ultrasound
import constants as const

class ProximitySensor():
    """ An object oriented interface for triggering and reading the left,
        center, and right ultrasound sensors simultaneously. Implements
        the Ultrasound object class.

        Arguments:
            trigpins: a tuple with the left, center, and right trigger pins
            echopins: a tuple with the left, center, and right echo pins
    """

    def __init__(self, io):
        self.sensors = (Ultrasound(io, const.L_ULTRASOUND_PINS[0],
                                   const.L_ULTRASOUND_PINS[1]),
                        Ultrasound(io, const.C_ULTRASOUND_PINS[0],
                                   const.C_ULTRASOUND_PINS[1]),
                        Ultrasound(io, const.R_ULTRASOUND_PINS[0],
                                   CONST.R_ULTRASOUND_PINS[1]))

    def trigger(self):
        """ triggers all ultrasound sensors """
        for i in range(len(self.sensor)):
            self.sensors[i].trigger()

    def read(self):
        """ returns the readings of ultrasound sensors (left, center, right)"""
        return (self.sensors[0].read(),
                self.sensors[1].read(),
                self.sensors[2].read())

def test():
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio unicron!")
        sys.exit(0)
    print("GPIO ready...")
    
    # Create objects
    sensor = ProximitySensor(io)
    while true:
        #Trigger the ultrasounds and wait 50 ms
        sensor.trigger()
        sleep(.05)

        #Read/Report
        reading = sensor.read()
        print("Distances = (%6.3f, %6.3f, %6.3f)" % reading)
