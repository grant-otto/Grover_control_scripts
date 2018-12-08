#! /usr/bin/python3

import numpy as np
import cv2
import os
import sys
from pathlib import Path
import os.path
import pyexiv2
import datetime
from dronekit import *
vehicle = connect('/dev/ttyS0', wait_ready=False, baud=921600)


def image_cap():
	cap = cv2.VideoCapture(0) #zero is first webcam attatched
	filename = '' #redefines the filename as an open string
	if cap.isOpened(): 	#checks if webcam is up and running
		ret, frame = cap.read() #capture single frame from the webcam to read
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
		date = datetime.datetime.utcnow().strftime('%Y-%m-%d-%H-%M-%S') #sets date to be current date/time in UTC
		filename = 'GrainImage_%s.png'%date #writes filename with UTC date & time
		path = "/home/pi/Grain_images/" # path file name to save to new directory on pi
		cv2.imwrite(os.path.join(path, filename), frame) #writes the frame to the designated path with the desired filename
		#print(filename) #sanity check to make sure filename is storing properly
		metadata = pyexiv2.ImageMetadata('/home/pi/Grain_images/'+filename) #calls for the metadata off of the image
		metadata.read() #reads the metadata
		metadata.modified = True #checks to see if the metadata can be modified
		metadata.writable = os.access('/home/pi/Grainimages/'+filename, os.W_OK) #makes sure the file in the defined path is readable for metadata
		key = 'Exif.Image.ImageDescription' #reference for saving the gps data in the exif tag
		lat = (vehicle.location.global_frame.lat) #sets lat to be the rover's latitude from dronekit
		lon = (vehicle.location.global_frame.lon) #sets Lon to be the rover's Longitude from dronekit
		alt = (vehicle.location.global_frame.alt) #sets Alt to be the rover's Altitude from dronekit
		value = '(%s, %s, %s)' %(lat,lon,alt) #takes Lat, Lon, and Alt and sets it to be the exif tag value
		metadata[key] = pyexiv2.ExifTag(key, value) #sets the key and value to the exif tag
		metadata.write() #writes the new tag to the metadata

	cap.release() #relases the camera function


if __name__ == "__main__":
	image_cap()