#! /usr/bin/python

'''
Grover: a grain size-measuring rover using Pixhawk 2.0 connected via a serial connection to a Raspberry Pi

Developed by Grant Otto, Courtney Cowger, Zach El-Azom, Eriq Gloria, Cole Stinger, Aaron Whitenight, and Lian Cheng Jin
University of Delaware Department of Mechanical Engineering
Faculty Advisor: Adam Wickenheiser

Developed for use at the Robotics Discovery Lab, University of Delaware School of Marine Science and Policy, CEOE
Sponsor and PI of Robotics Discovery Lab: Arthur Trembanis

wwww.udel.edu

All code is public and free to use.
'''

from dronekit import *
'''from dronekit import connect'''
vehicle = connect('/dev/ttyS0', wait_ready=True, baud=921600)
print("Hello, my name is Grover. The current firmware version is: ")
print vehicle.version


vehicle.mode=VehicleMode('HOLD')
print(vehicle.mode)
vehicle.mode=VehicleMode('MANUAL')
print(vehicle.mode)
