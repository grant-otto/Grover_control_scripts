#! /usr/bin/python
import time
from dronekit import *
'''from dronekit import connect'''
vehicle = connect('/dev/ttyS0', wait_ready=True, baud=921600)
print("Hello, my name is Grover. The current firmware version is: ")
print vehicle.version

while vehicle.mode==VehicleMode('AUTO'):
    print('auto')
    time.sleep(1)
while vehicle.mode==VehicleMode('MANUAL'):
    print('manual')
    time.sleep(1)
while vehicle.mode==VehicleMode('HOLD'):
    print('hold')
    time.sleep(1)
