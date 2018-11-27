#! /usr/bin/python3

'''
Grover: a grain size-measuring rover using Pixhawk 2.0 connected via a serial connecti$

Developed by Grant Otto, Courtney Cowger, Zach El-Azom, Eriq Gloria, Cole Stinger, Aar$
University of Delaware Department of Mechanical Engineering
Faculty Advisor: Adam Wickenheiser

Developed for use at the Robotics Discovery Lab, University of Delaware School of Mari$
Sponsor and PI of Robotics Discovery Lab: Arthur Trembanis

wwww.udel.edu

All code is public and free to use.
'''

'''
***** KNOWN ISSUES *****
- take image function needs to add in all code for taking the image, gps point, stepper control, etc
- if the switch for the image capture is left down, the vehicle will continuously take image after image.
	In the future, the r/c channel will be changed to a button but this will require a bit more troubleshooting
	on the controller.

'''

import time
from dronekit import *
'''from dronekit import connect'''
vehicle = connect('/dev/ttyS0', wait_ready=True, baud=921600)
print("Hello, my name is Grover. The current firmware version is: ")
print (vehicle.version)
print (vehicle.system_status.state)
print (vehicle.armed)
print (vehicle.mode)
print ('done checks')

while vehicle.mode==VehicleMode('MANUAL'):
	print('vehicle in manual')
while vehicle.mode==VehicleMode('AUTO'):
	print('vehicle in auto')
while vehicle.mode==VehicleMode('HOLD'):
	print('vehicle in hold')
	time.sleep(1)

