#! /usr/bin/python3

import numpy as np
import cv2
import os
import sys
from pathlib import Path
#import pyexiv2
import datetime
#from dronekit import connect, VehicleMode
#datetime.now.strftime("%m_%d_%Y %H:%M:%S")


def main_2():
	cap = cv2.VideoCapture(0)
	filename = ''

	if cap.isOpened():
		ret, frame = cap.read()
		#cv2.imshow('frame',frame)
	else:
		ret = False

	try:
		if ret:
			#%m_%d_%Y - month_day_year
			#filename = "GrainImage.png" #+ str(now) + ".png"
			filename = 'GrainImage_%s'%datetime.datetime.utcnow().strftime('%Y-%m-%d-%H-%M-%S')
			#path = 'C:/Users/Courtney/Docunments/Github/Grover_control_scripts'
			#cv2.imwrite(os.path.join(path, filename), frame)
			print(filename)
			cv2.imwrite(filename, frame)
			#filename+=new_filename
	except:
		pass

	cap.release()

if__name__ == "__main__":
	main_2()
