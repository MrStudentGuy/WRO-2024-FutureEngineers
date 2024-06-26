import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialising pins
LED = 17
BUTTON = 18

GPIO.setup(LED, GPIO.OUT)
GPIO.setup(BUTTON, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Function when button is pressed
def buttonpress(buttonstate):
    
    match buttonstate:
        case GPIO.HIGH:
            # If button is not pressed
            GPIO.output(LED, GPIO.LOW)
        case GPIO.LOW:
            # If button is pressed
            GPIO.output(LED, GPIO.HIGH)
            
# Turning LED off to start
GPIO.output(LED, GPIO.LOW)

# Ensures it is run only as a script, not an import
if __name__ == '__main__':
    print('Press the button to temporarily turn on the light')
    print('Ctrl + C to exit')

    try:
        while True:
            # Checking for button press
            buttonstate = GPIO.input(BUTTON)
            buttonpress(buttonstate)    
            
    except KeyboardInterrupt:
        # Cleans up GPIO pins / resets state when terminated using Ctrl + C
        GPIO.cleanup()
