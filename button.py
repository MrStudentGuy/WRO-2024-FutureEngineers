import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)

while True:
    lightstate = GPIO.input(18)
    match lightstate:
        case GPIO.HIGH:
            GPIO.output(17, GPIO.HIGH)
        case GPIO.LOW:
            GPIO.output(17, GPIO.LOW)

GPIO.cleanup()
