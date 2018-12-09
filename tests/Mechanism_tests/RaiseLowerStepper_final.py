#! /usr/bin/python


'''
Test script for final version of stepper raising and lowering that is implemented in Grover_main_v2.py
'''


import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)
GPIO.setup(40,GPIO.OUT) #Pin 40 is the GPIO out, can be easily changed
switchpin=33
GPIO.setup(switchpin, GPIO.IN,pull_up_down=GPIO.PUD_UP)
pwm = GPIO.PWM(40,50)
pwm.start(0)

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



cycles = stepperdown()
time.sleep(1)
print('take image')
time.sleep(1)
stepperup(cycles)
