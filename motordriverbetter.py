import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Set up motor driver pins
DIR_PIN = 27
PWM_PIN = 22
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(PWM_PIN, GPIO.OUT)

def clockwise():
    GPIO.output(27, GPIO.HIGH)
    print("CLOCKWISE")


def anti_clockwise():
    GPIO.output(27, GPIO.LOW)
    print("ANTI CLOCKWISE")


def toggle_motor_direction():
    motorstate = GPIO.input(17)
    if motorstate == 1:
        anti_clockwise()
    elif motorstate == 0:
        clockwise()


if __name__ == '__main__':
    print("Press the button to toggle the motor direction ")
    print("Ctrl+C to exit")

    while True:
        buttonstate = GPIO.input(18)
        if buttonstate == 0:
            toggle_motor_direction()
            time.sleep(0.175)

    GPIO.cleanup()
