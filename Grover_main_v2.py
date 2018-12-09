#! /usr/bin/python
#this script will NOT work in python3 due to dronekit's lack of compatibility with it.

'''
Grover: a grain size-measuring rover using Pixhawk 2.0 connected via a serial connection

Developed by Grant Otto, Courtney Cowger, Zach El-Azom, Eriq Gloria, Cole Stinger, Aaron Whitenight, Liancheng 
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

#######
'''Imports'''
#######
import RPi.GPIO as GPIO
import numpy as np
import cv2
import os
import sys
#from pathlib import Path
#import pyexiv2
import datetime
from dronekit import *
import time

#######
'''initialize gpio for stepper, wiping servo'''
#######
GPIO.setmode(GPIO.BOARD)
GPIO.setup(40,GPIO.OUT) #Pin 40 is the GPIO out, can be easily changed
pwm = GPIO.PWM(40,50)
pwm.start(0)

#######
'''Initialize GPIO for switch'''
#######
#GPIO.setwarnings(False)
switchpin=33
GPIO.setup(switchpin, GPIO.IN,pull_up_down=GPIO.PUD_UP)

#######
''' connect to vehicle and perform checks'''
#######
vehicle = connect('/dev/ttyS0', wait_ready=False, baud=921600)
#print("Hello, my name is Grover. The current firmware version is: ")
#print vehicle.version
print(vehicle.system_status.state)
print(vehicle.armed)
print(vehicle.mode)
print('done checks')


def image_save():	
    #######
    '''
    Executes a python3 script that will capture the image.
    This script captures and saves an image using the usb camera (microscope).
    It saves it to a datestamped filename, and adds the GPS information as text to the exif tag.
    '''
    #######	
    os.system("python3 /home/pi/Grover_control_scripts/tests/ImageCapture_tests/image_cap.py &")
    time.sleep(15)
def distance_to_current_waypoint():
    #######
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    #######
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1]									# commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

distancetopoint=distance_to_current_waypoint()
print(distancetopoint)

def stepperdown():
    '''
    lowers stepper until the switch is closed
    '''
    control_pins = [7,11,13,15]     												# Defines control pins

    for pin in control_pins:
        #print ("Setup pins")
        GPIO.setup(pin, GPIO.OUT)   												# Sets each control pin as an output
        GPIO.output(pin,0)         													# possibly set to GPIO.HIGH instead of 0

    lower_seq = [
        [1,0,0,1],
        [0,0,0,1],
        [0,0,1,1],
        [0,0,1,0],
        [0,1,1,0],
        [0,1,0,0],
        [1,1,0,0],
        [1,0,0,0]]

    cycles=0
    while True:
        if GPIO.input(switchpin)==True:
            break
        #Go through the sequence once. 1 rev = 8 cycles, gear reduction = 1/64, (8)(64) = 512 cycles
        #One revolution is 512 cycles = default
        for halfstep in range(8):
            #Go through each half step
            for pin in range(4):
                #Set each pin
                GPIO.output(control_pins[pin], (-1)*lower_seq[halfstep][pin])
                #Initially, halfstep=0 & pin=0, so control_pins[pin] = 7 and halfstep_seq[halfstep][pin] = 1
            time.sleep(0.001)
        cycles+=1
    return(cycles)


def stepperup(cycles):
    '''
    raises stepper one cycle more than the number of cycles it went down to lock it in place
    '''
    control_pins = [7,11,13,15]    								 					#Defines control pins

    for pin in control_pins:
        #print ("Setup pins")
        GPIO.setup(pin, GPIO.OUT)   												# Sets each control pin as an output
        GPIO.output(pin,0)          												# possibly set to GPIO.HIGH instead of 0

    raise_seq = [
        [1,0,0,0],
        [1,1,0,0],
        [0,1,0,0],
        [0,1,1,0],
        [0,0,1,0],
        [0,0,1,1],
        [0,0,0,1],
        [1,0,0,1]]

    for i in range(cycles+1):
        #Go through the sequence once. 1 rev = 8 cycles, gear reduction = 1/64, (8)(64) = 512 cycles
        #One revolution is 512 cycles = default
        for halfstep in range(8):
            #Go through each half step
            for pin in range(4):
                #Set each pin
                GPIO.output(control_pins[pin], raise_seq[halfstep][pin])
                #Initially, halfstep=0 & pin=0, so control_pins[pin] = 7 and seq[halfstep][pin] = 1
            time.sleep(0.001)

def SetAngle(ang):
    #######
    '''necessary for servo to work correctly'''
    #######
    duty = ang/18 + 2
    GPIO.output(40,True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(40,False)
    pwm.ChangeDutyCycle(0)


def wipe():
    #######
    ''' this function controls the servo to wipe the lens 5 times'''
    ######
    for i in range(5):
        SetAngle(30)
        SetAngle(180)

def take_image():
    #######
    '''
    This is the master function for taking an image. It controls the stepper motor going down, the image being taken,
    and the stepper coming back up.
    '''
    #######
    cycles = stepperdown()
    image_save()
    stepperup(cycles)
    wipe()

#######
''' MAIN LOOP '''
#######

while True: 																		# starts a perpetual loop any time the vehicle is connected
	while vehicle.mode==VehicleMode("AUTO"): 										# if the vehicle is in auto
		print('AUTO')	
		dist=distance_to_current_waypoint()
		while dist>1 and not dist == None:											# if the vehicle is more than a meter away from the current WP
			print('travelling to next waypoint...')
			print('distance to next waypoint: %f', dist)							# wait and display the distance to the next WP
			time.sleep(1)
			dist=distance_to_current_waypoint()										# reset the distance to the next WP and do again
		if dist<1 and not dist == None: 											# if the vehicle is less than a meter away from the current WP
			vehicle.mode = VehicleMode("HOLD") 										# put it on hold (on a rover, will stop the vehicle)
			time.sleep(3) 															# wait for the vehicle to come to a stop (3 seconds)
			print(vehicle.mode)
			print('take image')
			take_image()
			vehicle.mode = VehicleMode("AUTO") 										# put it back in AUTO
			print(vehicle.mode)
			time.sleep(15) 															# wait for 15 seconds so the vehicle can exit
		elif dist==None:
			print('At Home')
			time.sleep(1)
	while vehicle.mode==VehicleMode("MANUAL"): 										# if the vehicle is in MANUAL (remotely operated) mode:
		#print('MANUAL')
		if vehicle.channels['5'] < 1750: 											# if the switch by the H button on the Lightbridge is lowered
			vehicle.mode = VehicleMode("HOLD") 										# put the vehicle in HOLD (on a rover, will stop the vehicle)
			time.sleep(3)
			print(vehicle.mode) 													# wait for the vehicle to come to a stop
			take_image()															# take the image
			vehicle.mode = VehicleMode("MANUAL")									# put it back in manual
			time.sleep(3)															# wait a bit for the mode to change
			print(vehicle.mode)
		time.sleep(.5)
								# ***make sure the switch is immediately put back up after turning down
	while vehicle.mode==VehicleMode("HOLD"):										# if the vehicle is in hold, just chill and tell the computer you're holding
		print('holding')
		time.sleep(1)


