import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setup(26, GPIO.IN)

while True:
    if GPIO.input(26):
        print ("on")
    else:
        print ("off")

    time.sleep(1)

