"""
This file implements a ProximitySensor object which contains 3 Ultrasound
sensor. It provide functionality to trigger and read each of the sensors.

Authors: Edward Speer, Garrett Knuf
Date: 5/13/2023
"""

from sensing.drivers.ultrasound import Ultrasound
import constants as const
import pigpio
import time
import threading

class ProximitySensor():
    """ An object oriented interface for triggering and reading the left,
        center, and right ultrasound sensors simultaneously. Implements
        the Ultrasound object class.

        Arguments:
            trigpins: a tuple with the left, center, and right trigger pins
            echopins: a tuple with the left, center, and right echo pins
    """

    def __init__(self, io):
        self.io = io
        self.sensors = (Ultrasound(io, const.L_ULTRASOUND_PINS[0],
                                   const.L_ULTRASOUND_PINS[1]),
                        Ultrasound(io, const.C_ULTRASOUND_PINS[0],
                                   const.C_ULTRASOUND_PINS[1]),
                        Ultrasound(io, const.R_ULTRASOUND_PINS[0],
                                   const.R_ULTRASOUND_PINS[1]))
        print("Starting triggering thread...")
        self.triggering = True
        self.thread = threading.Thread(name="TriggerThread", target=self.run)
        self.thread.start()
        time.sleep(0.1) # Wait for the first measurements to arrive

    def trigger(self):
        """ triggers all ultrasound sensors """
        for i in range(len(self.sensors)):
            self.sensors[i].trigger()

    def run(self):
        while self.triggering:
            self.trigger()
            time.sleep(0.05)

    def shutdown(self):
        self.triggering = False
        print("Waiting for triggering thread to finish...")
        self.thread.join()
        print("Triggering thread returned.")

    def read(self):
        """ returns the readings of ultrasound sensors (left, center, right)"""
        return (self.sensors[0].read(),
                self.sensors[1].read(),
                self.sensors[2].read())
    
    def flight_time(self):
        return (self.sensors[0].flight_time(),
                self.sensors[1].flight_time(),
                self.sensors[2].flight_time())


def test():
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio unicron!")
        sys.exit(0)
    print("GPIO ready...")
    
    # Create objects
    sensor = ProximitySensor(io)
    while True:
        #Trigger the ultrasounds and wait 50 ms
        sensor.trigger()
        sleep(.05)

        #Read/Report
        reading = sensor.read()
        print("Distances = (%6.3f, %6.3f, %6.3f)" % reading)
