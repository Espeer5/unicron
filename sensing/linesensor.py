""" This file instantiates the IR class, a wrapper for 3 of the IR sensors 
    used in ME/CS/EE 129, providing basic functionality for reading the
    values of the 3 IR sensors system used for line following

    Authors: Edward Speer, Garrett Knuf
    Date: 4/16/2023 """
    
from sensing.drivers.ir import IR
import pigpio
import constants as const

class LineSensor():
	
	def __init__(self, io, IRPins):
		""" Instantiates three IR sensors on robot
		
			Arguments:
				io: pigpio interface
				IRPins: length 3 tuple with pins of (left, middle, right)
						IR sensors, respectively
			"""
		self.left_ir = IR(io, IRPins[0])
		self.middle_ir = IR(io, IRPins[1])
		self.right_ir = IR(io, IRPins[2])
		
	def read(self):
		""" Returns the state of all sensors as a length 3 tuple """
		return (self.left_ir.read(), self.middle_ir.read(), 
                self.right_ir.read())
		

def test_ls():
    """Tests and verifies the readings of the linesensor object by setting 
    up the connection to the IR sensor hardware and printing the readings taken 
    by the sensors continually.
    """
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio unicron!")
        sys.exit(0)
    print("GPIO ready...")
    ls = LineSensor(io, const.IR_PINS)
    while True:
        print(ls.read())
