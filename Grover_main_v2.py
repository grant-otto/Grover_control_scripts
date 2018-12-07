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
from pathlib import Path
import pyexiv2
import datetime
from dronekit import *
import time

#######
'''initialize gpio for stepper, steering servo'''
#######
GPIO.setmode(GPIO.BOARD)
GPIO.setup(40,GPIO.OUT) #Pin 40 is the GPIO out, can be easily changed
pwm = GPIO.PWM(40,50)
pwm.start(0)


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
	This script captures and saves an image using the usb camera (microscope).
	It saves it to a datestamped filename, and adds the GPS information as text to the exif tag.
	'''
	#######	
	cap = cv2.VideoCapture(0) 						#zero is first webcam attatched
	filename = ''
	if cap.isOpened(): 								#checks if webcam is up and running
		ret, frame = cap.read() 					#capture single frame from the webcam to read
		#cv2.imshow('frame',frame)
	else:
		ret = False

	if ret:
		'''GPS Exif Tag Description: A pointer to the GPS Info IFD. The interoperability
		structure of the GPS Info IFD, like that of Exif IFD, has no image data
		Blah
		Need to install pyexiv package
		pyexiv2 is a module that allows your python scripts to read and write data
		embedded in image files
		'''
		#%m_%d_%Y - month_day_year
		#filename = "GrainImage_.png" #+ str(now) + ".png"
		filename = 'GrainImage_%s.png'%datetime.datetime.utcnow().strftime('%Y-%m-%d-%H-%M-%S') #writes filename with UTC date & time
		#path = 'C:/Users/Courtney/Documents/Github/Grover_control_scripts'
		#cv2.imwrite(os.path.join(path, filename), frame)
		print(filename) 									#sanity check to make sure filename is storing properly
		cv2.imwrite(filename, frame) 						#writes the frame to an image file
		#filename+=new_filename
		metadata = pyexiv2.ImageMetadata(filename) 			#calls for the metadata off of the image
		metadata.read() 									#reads the metadata
		metadata.modified = True
		metadata.writable = os.access(filename, os.W_OK)
		key = 'Exif.Image.ImageDescription' 				#reference for saving the gps data in the exif tag
		lat = (vehicle.location.global_frame.lat)
		lon = (vehicle.location.global_frame.lon)
		alt = (vehicle.location.global_frame.alt)
		value = '(%s, %s, %s)' %(lat,lon,alt) 				#takes the gps data from dronekit
		metadata[key] = pyexiv2.ExifTag(key, value) 		#writes the key and value to the exif tag
		metadata.write()

	cap.release() 											#relases the camera function

if __name__ == "__main__":
	image_save()


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
    missionitem=vehicle.commands[nextwaypoint-1] 	#commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

distancetopoint=distance_to_current_waypoint()
print(distancetopoint)

def stepperdown():
	# tells the stepper to go down until it hits the switch
def stepperup():
	#tells the stepper to go back up

def SetAngle(ang):
	#######
	'''necessary for servo to work correctly'''
	#######
    duty = ang/18 + 2
    GPIO.output(40,True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(40,False)
    pwm.ChangeDutyCycle(0)


def wipe()
	#######
	''' this function controls the servo to wipe the lens 5 times'''
	#######
    SetAngle(30)
    SetAngle(180)
    SetAngle(30)
    SetAngle(180)
    SetAngle(30)
    SetAngle(180)
    SetAngle(30)
    SetAngle(180
    SetAngle(30)
    SetAngle(180)
    SetAngle(30)
    SetAngle(180)
    SetAngle(30)

def take_image():
	#######
	'''
	This is the master function for taking an image. It controls the stepper motor going down, the image being taken,
	and the stepper coming back up.
	'''
	#######

	'''
	from whiteboard code:
	while switch = LOW
                lower stepper
        stop stepper
        retrieve gps point
        take usb image
        modify exif tag to include gps point
        save to directory
        raise stepper #may need a switch for the top of the stepper

	'''
	stepperdown()
	take_image()
	stepperup()
	wipe()

#######
''' MAIN LOOP '''
#######

while True: 											# starts a perpetual loop any time the vehicle is connected
	while vehicle.mode==VehicleMode("AUTO"): 			# if the vehicle is in auto
		print('AUTO')	
		dist=distance_to_current_waypoint()
		while dist>1 and not dist == None:				# if the vehicle is more than a meter away from the current WP
			print('travelling to next waypoint...')
			print('distance to next waypoint: %f', dist)# wait and display the distance to the next WP
			time.sleep(1)
			dist=distance_to_current_waypoint()			# reset the distance to the next WP and do again
		if dist<1 and not dist == None: 				# if the vehicle is less than a meter away from the current WP
			vehicle.mode = VehicleMode("HOLD") 			# put it on hold (on a rover, will stop the vehicle)
			time.sleep(3) 								# wait for the vehicle to come to a stop (3 seconds)
			print(vehicle.mode)
			print('take image')
			take_image()
			vehicle.mode = VehicleMode("AUTO") 			# put it back in AUTO
			print(vehicle.mode)
			time.sleep(15) 								# wait for 15 seconds so the vehicle can exit
		elif dist==None:
			print('At Home')
			time.sleep(1)
	while vehicle.mode==VehicleMode("MANUAL"): 			# if the vehicle is in MANUAL (remotely operated) mode:
		#print('MANUAL')
		if vehicle.channels['5'] < 1750: 				# if the switch by the H button on the Lightbridge is lowered
			vehicle.mode = VehicleMode("HOLD") 			# put the vehicle in HOLD (on a rover, will stop the vehicle)
			time.sleep(3)
			print(vehicle.mode) 						# wait for the vehicle to come to a stop
			take_image()								# take the image
			vehicle.mode = VehicleMode("MANUAL")		# put it back in manual
			time.sleep(3)								# wait a bit for the mode to change
			print(vehicle.mode)
		time.sleep(.5)
								# ***make sure the switch is immediately put back up after turning down
	while vehicle.mode==VehicleMode("HOLD"):			# if the vehicle is in hold, just chill and tell the computer you're holding
		print('holding')
		time.sleep(1)


