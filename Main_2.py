#! /usr/bin/python3

import numpy as np
import cv2
import os
import sys
from pathlib import Path
#import pyexiv2
import datetime
#from dronekit import connect, VehicleMode
#datetime.now().strftime("%m_%d_%Y %H:%M:%S")


def main_2():
	cap = cv2.VideoCapture(0) 							#zero is first webcam attatched
	filename = ''

	if cap.isOpened(): 								#checks if webcam is up and running
		ret, frame = cap.read() 						#capture single frame from the webcam to read
		#cv2.imshow('frame',frame)
	else:
		ret = False

	try:
		if ret:
			'''GPS Exif Tag Description: A pointer to the GPS Info IFD. The interoperability
			structure of the GPS Info IFD, like that of Exif IFD, has no image data
			Need to install pyexiv package
			pyexiv2 is a module that allows your python scripts to read and write data
			embedded in image files
			'''
			#%m_%d_%Y - month_day_year
			#filename = "GrainImage_.png" #+ str(now) + ".png"
			filename = 'GrainImage_%s'%datetime.datetime.utcnow().strftime('%Y-%m-%d-%H-%M-%S') #writes filename with UTC date & time
			#path = 'C:/Users/Courtney/Documents/Github/Grover_control_scripts'
			#cv2.imwrite(os.path.join(path, filename), frame)
			print(filename) #sanity check to make sure filename is storing properly
			cv2.imwrite(filename, frame) #writes the frame to an image file
			#filename+=new_filename
		except:
			pass

	cap.release() #relases the camera function

if __name__ == "__main__":
	main_2()
