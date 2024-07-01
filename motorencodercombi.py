import RPi.GPIO as GPIO
import gpiozero
from signal import pause
import time

# Motor pins
DIR = 27
PWM = 22

# Encoder pins
CHANNEL_A_PIN = 23
CHANNEL_B_PIN = 24  # Changed from 27 to avoid conflict with DIR pin

# Constants
BOUNCE_TIME = 100

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor setup
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(PWM, GPIO.OUT)
GPIO.output(DIR, GPIO.HIGH)  # Set motor to clockwise rotation
pwm = GPIO.PWM(PWM, 100)  # Initializing PWM at 100Hz
pwm.start(0)  # Start at 0% duty cycle / motor off

# Encoder setup
encoder_a = gpiozero.DigitalInputDevice(CHANNEL_A_PIN)
encoder_b = gpiozero.DigitalInputDevice(CHANNEL_B_PIN)

# Initialize the counter
counter = 0

def encoder_counter():
    global counter
    if encoder_b.value:
        counter += 1
    else:
        counter -= 1
    print("Encoder count:", counter)

# Set up the encoder pins to trigger on rising edges
encoder_a.when_activated = encoder_counter

print("Motor control and encoder counter started. Enter duty cycle in %")
print("Ctrl+C to exit")

try:
    while True:
        # Takes duty cycle input
        speed = int(input("Enter duty cycle (0-100): "))

        # Checks for invalid input
        if speed < 0 or speed > 100:
            print('Invalid input')
            continue

        # Sets duty cycle of motor
        pwm.ChangeDutyCycle(speed)
        print(f"Motor speed set to {speed}%")

except KeyboardInterrupt:
    print("\nProgram terminated.")
    print("Final encoder count:", counter)
    # Clean up GPIO pins
    pwm.stop()
    GPIO.cleanup()