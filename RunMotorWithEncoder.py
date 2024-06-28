import RPi.GPIO as GPIO
import gpiozero
import time

# Constants
CHANNEL_A_PIN = 23
CHANNEL_B_PIN = 27
DIR = 22
PWM = 26

# GPIO setup
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(PWM, GPIO.OUT)

# Define the encoder pins
encoder_a = gpiozero.DigitalInputDevice(CHANNEL_A_PIN)
encoder_b = gpiozero.DigitalInputDevice(CHANNEL_B_PIN)

# Initialize the counter
counter = 0

# Sets motor to clockwise rotation
GPIO.output(DIR, GPIO.HIGH)

def encoder_counter():
    global counter
    if encoder_b.value:
        counter += 1
    else:
        counter -= 1
    print("Counter:", counter)

# Set up the encoder pins to trigger on rising edges
encoder_a.when_activated = encoder_counter

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

print("Encoder counter started. Press 't' to toggle the motor on/off. Press 'q' to quit.")

try:
    while True:
        # Non-blocking input
        user_input = input()

        if user_input.lower() == 't':
            toggle_motor()
        elif user_input.lower() == 'q':
            raise KeyboardInterrupt

        # Small delay to reduce CPU usage
        time.sleep(0.01)

except KeyboardInterrupt:
    print("Program stopped.")
    print("Final counter value:", counter)
    GPIO.cleanup()