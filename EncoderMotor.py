import RPi.GPIO as GPIO
import gpiozero
from signal import pause
import time

# Constants
CHANNEL_A_PIN = 23
CHANNEL_B_PIN = 27
BOUNCE_TIME = 100

# Define the encoder pins
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
    print("Counter:", counter)

# Set up the encoder pins to trigger on rising edges
encoder_a.when_activated = encoder_counter

print("Encoder counter started. Press Ctrl+C to exit.")

try:
    pause()
except KeyboardInterrupt:
    print("Encoder counter stopped.")
    print("Final counter value:", counter)