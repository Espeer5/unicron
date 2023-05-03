"""
This file contains a filtering algorithm and a series of detectors which employ that 
filtering algorithm to accurate detect events which the robot needs to act on.
The purpose of the filtering is to screen out sensor noise such that the robot 
only acts on real events and does not pick up phantom events.

Authors: Edward Speer, Garrett Knuf
Date: 4/24/23
"""

import linesensor as ls
import time


class Filters:
    """
    This class provides a filtering algorithm which acts on the 
    raw IR sensor data to provide filtered outputs of the robot's 
    linesensor.

    Inputs: linesensor: a linsesnor object providing IR data
            T: time constant specifying filter sensitivity
    """

    # Filter thresholds specifying what values indicate a true event
    FILTER_MIN_THRESHOLD = 0.2
    FILTER_MAX_THRESHOLD = 0.8

    def __init__(self, linesensor, T):
        self.linesensor = linesensor
        self.filteredVals = [val for val in linesensor.read()]
        self.T = T
        self.last_time = time.time()
        self.last_state = [val for val in linesensor.read()]

    def update(self):
        """ Applies the filtering algorithm to each IR sensor, 
            detecting how much time has passed since the previous 
            update
        """
        dt = time.time() - self.last_time
        self.filteredVals = [self.filteredVals[i] + (dt / self.T) *
                (self.linesensor.read()[i] - self.filteredVals[i])
                for i in range(len(self.filteredVals))]
        self.last_time = time.time()

    def get(self):
        """ returns a list with filtered binary states of line sensor
            Filtered values are sent to 1 if greater than the 
            filter threshold, 0 otherwise."""
        self.update()
        output = []
        for i in range(len(self.filteredVals)):
            if self.filteredVals[i] >= self.FILTER_MAX_THRESHOLD:
                output.append(1)
            elif self.filteredVals[i] <= self.FILTER_MIN_THRESHOLD:
                output.append(0)
            else:
                output.append(self.last_state[i])
            self.last_state[i] = output[i]
        return output



class InterDetector:
    """ Intersection detector. Applies a filtering algorithm 
        to each IR sensor, to determine if all sensors are truly 
        high.
        
        Inputs: linesensor: a linsensor object for IR input
                T: time sensitivity constant for filtering
    """

    def __init__(self, linesensor, T):
        self.filters = Filters(linesensor, T)

    def update(self):
        """ Updates the filtered values of each sensor
        """
        self.filters.update()

    def check(self):
        """ return 1 is an intersection is detected, 0 otherwise """
        if self.filters.get() == [1, 1, 1]:
            return 1
        else:
            return 0


class LRDetector:
    """ Leave-the-road / line following detector. Filters IR data 
    to determine if the robot has truly left the line and screens 
    out noise.

    Inputs: linesensor: a linesensor object for IR input
            T: time sensitivity constant for filtering
    """

    # The relative significance of possible sensor readings
    position_weights = {(0, 1, 0): 0, (1, 0, 1): 0,
                        (0, 0, 1): 1, (0, 1, 1): 0.5,
                        (1, 0, 0): -1, (1, 1, 0): -0.5,
                        (1, 1, 1): 0, (0, 0, 0): 0}

    # Filtered values above this threshold indicate line departure
    THRESHOLD = 0.5

    def __init__(self, linesensor, T):
        self.filters = Filters(linesensor, T)
        self.linesensor = linesensor
        self.buffer = 0
        self.T = T
        self.last_time = time.time()

    def update(self):
        """ update the filter based on time elapsed and importance of sensor
            reading """
        reading = tuple(self.filters.get())
        if self.position_weights[reading] != 0:
            dt = time.time() - self.last_time
            self.buffer += (dt / self.T) * (self.position_weights[reading]
                                    - self.buffer)
        self.last_time = time.time()

    def get(self):
        """
        gets the filtered position of robot and returns the following
            1: LEFT
            0: STRAIGHT
            -1: RIGHT
        """
        self.update()
        if self.buffer >= self.THRESHOLD:
            return 1    # left
        elif self.buffer <= -self.THRESHOLD:
            return -1   # right
        else:
            return 0    # center


class NextRoadDetector:
    """ Assuming robot positioned at intersection and is turning,
        determines when the next road is found by appplying filtering 
        to each sensor to ensure that noise does not falsely indicate
        the location of the next road.
        
        Inputs: linesensor: a linesensor object for IR input
                T: Time sensitivity constant for filtering
                Direction: Direction robot is turning (l/r)
    """

    # Filtered values over this threshold indicate road found
    THRESHOLD = 0.8

    # Aliases for the tuple indices of each position sensor
    sense_map = {"LEFT": 0, "CENTER": 1, "RIGHT": 2}

    def __init__(self, linesensor, T, direction):
        self.filters = Filters(linesensor, T)
        self.linesensor = linesensor
        self.T = T
        self.direction = direction
        self.last_time = time.time()
        self.buffer = 0
        self.active = False

    def update(self):
        """ Applies the filtering algorithm to each reading to screen out 
        noise based on how much time has elapsed.
        """
        if self.active:
            reading = self.filters.get()[self.sense_map[self.direction]]
            dt = time.time() - self.last_time
            self.buffer += (dt / self.T) * (reading - self.buffer)
            self.last_time = time.time()
        else:
            if self.filters.get()[self.sense_map["CENTER"]] == 0:
                self.active = True
        
    def found_road(self):
        """ Returns a boolean indicating if the filtered sensors indicate 
        the next road has been found.
        """
        self.update()
        return self.buffer >= self.THRESHOLD
