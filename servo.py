import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialising pins
PWM = 17

GPIO.setup(PWM, GPIO.OUT)

# Motor control function
pwm = GPIO.PWM(PWM, 100)  # Initialising PWM at 100Hz
pwm.start(0)  # Start at 0% duty cycle / servo off

# Ensures it is run only as a script, not an import
if __name__ == '__main__':
    print("Enter angle in degrees between 0 and 180")
    print("Ctrl+C to exit")

    try:
        while True:
            angle = int(input())
            pwm.ChangeDutyCycle(angle)

    except KeyboardInterrupt:
        # Cleans up GPIO pins / resets state and terminates PWM when terminated using Ctrl + C
        pwm.stop()
        GPIO.cleanup()