import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialising pins
DIR = 27
PWM = 22

GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(PWM, GPIO.OUT)

# Sets motor to clockwise rotation
GPIO.output(DIR, GPIO.HIGH)

# Motor control function
pwm = GPIO.PWM(PWM, 100)  # Initialising PWM at 100Hz
pwm.start(0)  # Start at 0% duty cycle / motor off


# Ensures it is run only as a script, not an import
if __name__ == '__main__':
    print("Enter a number ")
    print("Ctrl+C to exit")

    try:
        while True:
            speed = int(input())
            pwm.start(speed)

    except KeyboardInterrupt:
        # Cleans up GPIO pins / resets state when terminated using Ctrl + C
        GPIO.cleanup()
