import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialising pins
LED = 17
button = 18

GPIO.setup(LED, GPIO.OUT)
GPIO.setup(button, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Turning LED off to start
GPIO.output(LED, GPIO.LOW)

try:
    while True:
        # Checking for button press
        buttonstate = GPIO.input(18)

        match buttonstate:
            case GPIO.HIGH:
                # If button is not pressed
                GPIO.output(17, GPIO.LOW)
            case GPIO.LOW:
                # If button is pressed
                GPIO.output(17, GPIO.HIGH)
except KeyboardInterrupt:
    GPIO.cleanup()
