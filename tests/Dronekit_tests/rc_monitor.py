#! /usr/bin/python
import time
from dronekit import *
'''from dronekit import connect'''
vehicle = connect('/dev/ttyS0', wait_ready=True, baud=921600)
print("Hello, my name is Grover. The current firmware version is: ")
print vehicle.version
while True:
        print(vehicle.channels['5'])
        print(vehicle.channels['6'])
        print(vehicle.channels['7'])
        print(vehicle.channels['8'])
	print(' ')
	time.sleep(.5)
