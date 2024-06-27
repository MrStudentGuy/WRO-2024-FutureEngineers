import RPi.GPIO as GPIO
import gpiozero
from signal import pause
import time

# Constants
CHANNEL_A_PIN = 23
CHANNEL_B_PIN = 27
BOUNCE_TIME = 100


# Define the encoder pins
encoder_a = gpiozero.DigitalInputDevice(23)  # Pin 23 for encoder A
encoder_b = gpiozero.DigitalInputDevice(27)  # Pin 27 for encoder B

# Initialize the counter
counter = 0

def encoder_a_rise():
    global counter
    counterA = 0
    counterB = 0


    if encoder_b.value:
        counterA += 1
    else:
        counterB -= 1

   # print("A: ", counterA, "B: ", counterB)

# def encoder_counter():
#     global counter
#     if encoder_a.value:
#         counter += 1
#     else:
#         counter -= 1

def encoder_b_rise():
    global counter

    counterC = 0
    counterD = 0

    if encoder_a.value:
        counterC -= 1

    else:
        counterD += 1

    #print("C: ", counterC, "D: ", counterD)


# Set up the encoder pins to trigger on rising edges
encoder_a.when_activated = encoder_a_rise
encoder_b.when_activated = encoder_b_rise

print("Encoder counter started. Press Ctrl+C to exit.")

try:
    global counterA
    global counterB
    global counterC
    global counterD
    while True:
        time.sleep(0.1)
        # print("counter A: {}".format(counter)
        print("A : {}, B: {}, C:{}, D: {}".format(counterA, counterB, counterC, counterD))
except KeyboardInterrupt:
    print("Encoder counter stopped.")
    print("Final counter value:", counter)