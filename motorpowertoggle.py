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


# Turns off motor
def motor_off():
    GPIO.output(PWM, GPIO.LOW)
    print("MOTOR OFF")


# Turns on motor
def motor_on():
    GPIO.output(PWM, GPIO.HIGH)
    print("MOTOR ON")


def toggle_motor():
    # Checks motor state
    motorstate = GPIO.input(PWM)

    # Flips motor state
    if motorstate == 1:
        motor_off()
    elif motorstate == 0:
        motor_on()


# Ensures it is run only as a script, not an import
if __name__ == '__main__':
    print("Press the button to turn the motor on and off")
    print("Ctrl+C to exit")

    try:
        while True:
            # Sets motor to clockwise rotation
            GPIO.output(DIR, GPIO.HIGH)

            # Detects button press
            buttonstate = GPIO.input(button)

            # If button is pressed, calls motor toggle function
            if buttonstate == 0:
                toggle_motor()
                time.sleep(0.175)  # Debounce delay

    except KeyboardInterrupt:
        # Cleans up GPIO pins / resets state when terminated using Ctrl + C
        GPIO.cleanup()
