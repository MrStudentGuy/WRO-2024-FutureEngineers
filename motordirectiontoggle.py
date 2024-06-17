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

# Ensures it is run as a script, not as an import
if __name__ == '__main__':
    print("Press the button to toggle the motor's direction")
    print("Ctrl+C to exit")
    
    try:
        while True
            # Turns on motor
            GPIO.output(PWM, GPIO.HIGH)

            # Detects button press
            buttonstate = GPIO.input(18)

            # If button is pressed, calls direction toggle function
            if buttonstate == 0:
                toggle_motor_direction()
                time.sleep(0.175) # Debounce delay
                
    except KeyboardInterrupt:
        GPIO.cleanup()
