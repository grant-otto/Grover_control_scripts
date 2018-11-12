#! /usr/bin/python3

import numpy as np
import cv2
import os
#import pyexiv2
import datetime
#from dronekit import connect, VehicleMode
#datetime.now().strftime("%m_%d_%Y %H:%M:%S")


def main():
    cap = cv2.VideoCapture(0) 							#zero is first webcam attatched
    n = 0
    
    if cap.isOpened(): 								#checks if webcam is up and running
        ret, frame = cap.read() 						#capture single frame from the webcam to read
        #cv2.imshow('frame',frame)
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
        #filename = "GrainImage_.png" #+ str(now) + ".png"
        filename = 'GrainImage_%s.png'%datetime.datetime.utcnow().strftime('%Y-%m-%d-%H-%M-%S') #writes filename with UTC date & time
        #path = 'C:/Users/Courtney/Documents/Github/Grover_control_scripts'
        #cv2.imwrite(os.path.join(path, filename), frame)
        #print(filename) #sanity check to make sure filename is storing properly
        cv2.imwrite(filename, frame) #writes the frame to an image file
        n+=1
        
    cap.release() #relases the camera function

if __name__ == "__main__":
    main()
