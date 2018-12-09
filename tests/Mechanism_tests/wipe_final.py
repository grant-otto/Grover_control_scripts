#! /usr/bin/python

'''
Test script for final version of wiper code that is implemented in Grover_main_v2.py
'''


import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)
GPIO.setup(40,GPIO.OUT) #Pin 40 is the GPIO out, can be easily changed
pwm = GPIO.PWM(40,50)
pwm.start(0)

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


def wipe():
    #######
    ''' this function controls the servo to wipe the lens 5 times'''
    ######
    SetAngle(30)
    SetAngle(180)


wipe()
time.sleep(5)
wipe()
print('done')
