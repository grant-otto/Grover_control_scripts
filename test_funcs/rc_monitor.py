#! /usr/bin/python

from dronekit import *
'''from dronekit import connect'''
vehicle = connect('/dev/ttyS0', wait_ready=True, baud=921600)
print("Hello, my name is Grover. The current firmware version is: ")
print vehicle.version

switch=vehicle.channels['8']
print(switch)
