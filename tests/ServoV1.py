import RPi.GPIO as GPIO
from time import sleep
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11, GPIO.OUT)
pwm= GPIO.PWM(11,50)
pwm.start(0)
n=0
def SetAngle(angle):
    duty = angle/18 + 2
    GPIO.output(11, True)
    pwm.ChangeDutyCycle(duty)
    sleep(1)
    GPIO.output(11,False)
    pwm.ChangeDutyCycle(0)

while (n<200):

    SetAngle(210)
    SetAngle(30)
    n+=1

pwm.stop()

print("finished")

GPIO.cleanup()
