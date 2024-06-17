import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def led_toggle(currentstate):
    global laststate
    newstate = currentstate ^ laststate
    GPIO.output(17, newstate)
    laststate = currentstate


if __name__ == '__main__':
    
    laststate = False

    while True:
        if GPIO.input(18) == True:
            currentstate = True
            led_toggle(currentstate)
    GPIO.cleanup()
