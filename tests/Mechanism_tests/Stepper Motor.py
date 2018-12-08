import RPi.GPIO as GPIO
import time

# Variables

delay = .0055
steps = 500

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Enable pins for IN1-4 to control step sequence

coil_A_1_pin = 18
coil_A_2_pin = 23
coil_B_1_pin = 24
coil_B_2_pin = 25

# Set pin states

GPIO.setup(coil_A_1_pin,GPIO.OUT)
GPIO.setup(coil_A_2_pin,GPIO.OUT)
GPIO.setup(coil_B_1_pin,GPIO.OUT)
GPIO.setup(coil_B_2_pin,GPIO.OUT)

# Function for step sequence

def setStep(w1, w2, w3, w4):
    GPIO.output(coil_A_1_pin,w1)
    GPIO.output(coil_A_2_pin,w2)
    GPIO.output(coil_B_1_pin,w3)
    GPIO.output(coil_B_2_pin,w4)

# Loop through step sequence based on number of steps

for i in range(0, steps):

    setStep(1,0,1,0)
    time.sleep(delay)
    setStep(0,1,1,0)
    time.sleep(delay)
    setStep(0,1,0,1)
    time.sleep(delay)
    setStep(1,0,0,1)
    time.sleep(delay)

#Reverse previous step sequence to reverse direction of the motor

for i in range(0,steps):

    setStep(1,0,0,1)
    time.sleep(delay)
    setStep(0,1,0,1)
    time.sleep(delay)
    setStep(0,1,1,0)
    time.sleep(delay)
    setStep(1,0,1,0)
    time.sleep(delay)
    
##StepCount1 = 4
##Seq1 = []
##Seq1 = range(0, stepCount1)
##Seq1[0] = [1,0,0,0]
##Seq1[1] = [0,1,0,0]
##Seq1[2] = [0,0,1,0]
##Seq1[3] = [0,0,0,1]
##
##StepCount2 = 8
##Seq2 = []
##Seq2 = range(0, stepCount2)
##Seq2[0] = [1,0,0,0]
##Seq2[1] = [1,1,0,0]
##Seq2[2] = [0,1,0,0]
##Seq2[3] = [0,1,1,0]
##Seq2[4] = [0,0,1,0]
##Seq2[5] = [0,0,1,1]
##Seq2[6] = [0,0,0,1]
##Seq2[7] = [1,0,0,1]
##
##Seq = Seq2
##StepCount = StepCount2
##
##while 1==1:
##for pin in range(0,8):
##    xpin = StepPins[pin]
##    if Seq[StepCounter][pin}!=0:
##        print "Step %iEnable%i" %(StepCounter,xpin)
##        GPIO.output(xpin, True)
##    else:
##        GPIO.output(xpin, False)
##        StepCounter += 1
##
###If reach the end of the sequencde start again\
##
##if(StepCounter==StepCount):
##    StepCounter=0
##
##if (StepCounter<0):
##    StepCounter = StepCount
##
###Wait before moving on
##
##time.sleep(WaitTime)
