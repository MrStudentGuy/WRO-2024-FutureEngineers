import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialising pins 
button = 18
DIR = 27
PWM = 22

GPIO.setup(button, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(PWM, GPIO.OUT)

# Turns on motor
GPIO.output(PWM, GPIO.HIGH)

# Motor rotates clockwise
def clockwise():
    GPIO.output(DIR, GPIO.HIGH)
    print("CLOCKWISE")

# Motor rotates anti-clockwise
def anti_clockwise():
    GPIO.output(DIR, GPIO.LOW)
    print("ANTI CLOCKWISE")


def toggle_motor_direction():
    # Checks motor direction
    direction = GPIO.input(DIR)

    #Flips motor direction
    if direction == 1:
        anti_clockwise()
    elif direction == 0:
        clockwise()

# Ensures it is run only as a script, not an import
if __name__ == '__main__':
    print("Press the button to toggle the motor's direction")
    print("Ctrl+C to exit")
    
    try:
        while True:
            # Detects button press
            buttonstate = GPIO.input(button)

            # If button is pressed, calls direction toggle function
            if buttonstate == 0:
                toggle_motor_direction()
                time.sleep(0.175) # Debounce delay
                
    except KeyboardInterrupt:
        # Cleans up GPIO pins / resets state when terminated using Ctrl + C
        GPIO.cleanup()
