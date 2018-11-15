import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.IN,pull_up_down=GPIO.PUD_UP)
  
print('before while loop')

while True:
    inputValue = GPIO.input(18)
    print(inputValue)
    if (inputValue == True):
        print("Button press")
    time.sleep(0.3)
