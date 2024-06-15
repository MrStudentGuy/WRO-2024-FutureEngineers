import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17, GPIO.OUT)

while True:
    lightstate = int(input())
    match lightstate:
        case 1:
            GPIO.output(17, GPIO.HIGH)
        case 0:
            GPIO.output(17, GPIO.LOW)

GPIO.cleanup()
