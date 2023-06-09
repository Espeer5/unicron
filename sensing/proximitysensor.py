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
import sys
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
        #print("Starting triggering thread...")
        self.triggering = True
        self.thread = threading.Thread(name="TriggerThread", target=self.run)
        self.channel = 0
        self.thread.start()
        time.sleep(0.1) # Wait for the first measurements to arrive
        

    def trigger(self):
        """ triggers all ultrasound sensors """
        self.channel = (self.channel + 1) % 3
        self.sensors[self.channel].trigger()

    def run(self):
        """Triggers the ultrasounds in an infinite loop such that readings 
        are taken constantly at the maximum safe frequency.
        """
        while self.triggering:
            self.trigger()
            time.sleep(0.05)

    def shutdown(self):
        """Shuts down the ultrasound thread so that the infinite loop is closed, 
        and the thread is joined back to the main thread.
        """
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
        """Returns the time between trigger and reception for the signal sent 
        out by each sensor.
        """
        return (self.sensors[0].flight_time(),
                self.sensors[1].flight_time(),
                self.sensors[2].flight_time())


def test():
    """Tests the ultrasounds sensors by printing the measurements the sensors 
    are taking as they're taken in order to verify that the measurements are 
    accurate and taken with the appropriate frequency.
    """
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio unicron!")
        sys.exit(0)
    print("GPIO ready...")
    
    # Create objects
    sensor = ProximitySensor(io)

    try:
        while True:
            # Trigger the ultrasounds and wait 50 ms
            time.sleep(.08)

            #Read/Report
            reading = sensor.read()
            print("Distances = (%6.3f, %6.3f, %6.3f)" % reading)
    except BaseException as e:
        print("exiting...")
    sensor.shutdown()
    io.stop()
