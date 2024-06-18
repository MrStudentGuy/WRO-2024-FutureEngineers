import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialising pins
LED = 17
BUTTON = 18

GPIO.setup(LED, GPIO.OUT)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Setting initial state of button and/or lightbulb
previousstate = 0
currentstate = 0


# Turns on LED
def led_on():
    GPIO.output(LED, GPIO.HIGH)
    print("LED ON")


# Turns off LED
def led_off():
    GPIO.output(LED, GPIO.LOW)
    print("LED OFF")


# Checks current state of LED, flips it
def toggle_led():
    ledstate = GPIO.input(LED)
    if ledstate == 1:
        led_off()
    else:
        led_on()


# Ensures it is run only as a script, not an import
if __name__ == '__main__':
    print("Press the button to toggle the light")
    print("Ctrl+C to exit")

    try:
        while True:
            # Detects button press
            currentstate = GPIO.input(BUTTON)

            # Calls function when button is released
            if previousstate == 1 and currentstate == 0:
                toggle_led()

            previousstate = currentstate

    except KeyboardInterrupt:
        # Cleans up GPIO pins / resets state when terminated using Ctrl + C
        GPIO.cleanup()
