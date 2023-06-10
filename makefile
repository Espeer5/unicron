#Gives the user the ability to either run the main robot code, or perform 
# various hardware tests on the robot to verify hardware functionality
#
#Authors: Edward Speer, Garrett Knuf
#Date: 6/10/23

#Run the main robot code. Start up the gui and operate the robot
run:
	@python3 run.py

#Test the motor hardware by driving a pattern of prescribed actions
motor_test:
	@python3 test.py Motors

#Test the IR sensor hardware by viewing the reading values in the terminal
sensor_test:
	@python3 test.py IRSensors

#Test the ultrasound hardware by viewing the reading values in the terminal
ultrasound_test:
	@python3 test.py Ultrasounds
