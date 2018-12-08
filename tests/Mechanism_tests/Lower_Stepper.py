### CCW Rotation of stepper ###

##import RPi.GPIO as GPIO
##import time
##
##
##GPIO.setwarnings(False)
##GPIO.setmode(GPIO.BOARD)

def Lower_Stepper(control_pins,lower_seq):

    import RPi.GPIO as GPIO
    import time

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BOARD)


    control_pins = [7,11,13,15]     #Defines control pins

    for pin in control_pins:
        #print ("Setup pins")
        GPIO.setup(pin, GPIO.OUT)   #Sets each control pin as an output
        GPIO.output(pin,0)          # possibly set to GPIO.HIGH instead of 0

    lower_seq = [
        [1,0,0,1],
        [0,0,0,1],
        [0,0,1,1],
        [0,0,1,0],
        [0,1,1,0],
        [0,1,0,0],
        [1,1,0,0],
        [1,0,0,0]]

    for i in range(512):
        #Go through the sequence once. 1 rev = 8 cycles, gear reduction = 1/64, (8)(64) = 512 cycles
        #One revolution is 512 cycles = default
        for halfstep in range(8):
            #Go through each half step
            for pin in range(4):
                #Set each pin
                GPIO.output(control_pins[pin], (-1)*lower_seq[halfstep][pin])
                #Initially, halfstep=0 & pin=0, so control_pins[pin] = 7 and halfstep_seq[halfstep][pin] = 1
            time.sleep(0.001)


    GPIO.cleanup()

