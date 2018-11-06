#! /usr/bin/python3

import cv2
import os
import pyexiv2
#from datetime import *
#import datetime
from datetime import date
from datetime import time
from datetime import datetime
from datetime import tzinfo
from dronekit import connect, VehicleMode
datetime.now().strftime("%m_%d_%Y %H:%M:%S")


def main():
    cap = cv2.VideoCapture(0) 							#zero is first webcam attatched
    n = 0
    now = datetime.now()
    
    if cap.isOpened(): 								#checks if webcam is up and running
        ret, frame = cap.read() 						#capture single frame from the webcam to read
    else:
        ret = False

    if ret:
        '''GPS Exif Tag Description: A pointer to the GPS Info IFD. The interoperability
            structure of the GPS Info IFD, like that of Exif IFD, has no image data
        Need to install pyexiv package
        pyexiv2 is a module that allows your python scripts to read and write data
            embedded in image files
	'''
        #metadata = pyexiv2.ImageMetadata(frame) 				#calls for the metadata off of the image
        #metadata.read() 							#reads the metadata
        key = 'Exif.Image.GPSTag' 						#reference for saving the gps data in the exif tag
        #value = dronekit.LocationGlobal 					#takes the gps data from dronekit
        #metadata[key] = pyexiv2.ExifTag(key, value) 				#writes the key and value to the exif tag
        #%m_%d_%Y - month_day_year
        #%n - image number
        filename = "GrainImage_" + str(datetime.now())    #.strftime("%m_%d_%Y %H:%M:%S") + ".jpg" %n
        path = '/home/grant/GrainSizeImages'
        cv2.imwrite(os.path.join(path, filename), frame)
        #cv2.imwrite(filename, frame) #writes the frame to an image file
        n+=1
        
    cap.release() #relases the camera function

if __name__ == "__main__":
    main()
