# import RPi.GPIO as GPIO
# import gpiozero
# from signal import pause
# import time
#
# # Motor pins
# DIR = 27
# PWM = 22
#
# # Encoder pins
# CHANNEL_A_PIN = 23
# CHANNEL_B_PIN = 24  # Changed from 27 to avoid conflict with DIR pin
#
# # Constants
# BOUNCE_TIME = 100
#
# # Set up GPIO
# GPIO.setmode(GPIO.BCM)
# GPIO.setwarnings(False)
#
# # Motor setup
# GPIO.setup(DIR, GPIO.OUT)
# GPIO.setup(PWM, GPIO.OUT)
# GPIO.output(DIR, GPIO.HIGH)  # Set initial direction to clockwise
# pwm = GPIO.PWM(PWM, 100)  # Initializing PWM at 100Hz
# pwm.start(0)  # Start at 0% duty cycle / motor off
#
# # Encoder setup
# encoder_a = gpiozero.DigitalInputDevice(CHANNEL_A_PIN)
# encoder_b = gpiozero.DigitalInputDevice(CHANNEL_B_PIN)
#
# # Initialize the counter
# counter = 0
#
# def encoder_counter():
#     global counter
#     if encoder_b.value:
#         counter += 1
#     else:
#         counter -= 1
#     print("Encoder count:", counter)
#
# # Set up the encoder pins to trigger on rising edges
# encoder_a.when_activated = encoder_counter
#
# print("Motor control and encoder counter started.")
# print("Enter: 0 for no power, 1 for 50% clockwise, -1 for 50% anticlockwise")
# print("Ctrl+C to exit")
#
# try:
#     while True:
#         # Takes input
#         user_input = int(input("Enter command (0, 1, or -1): "))
#
#         if user_input == 0:
#             pwm.ChangeDutyCycle(0)
#             print("Motor stopped")
#         elif user_input == 1:
#             GPIO.output(DIR, GPIO.HIGH)  # Set direction to clockwise
#             pwm.ChangeDutyCycle(50)
#             print("Motor running at 50% duty cycle clockwise")
#         elif user_input == -1:
#             GPIO.output(DIR, GPIO.LOW)  # Set direction to anticlockwise
#             pwm.ChangeDutyCycle(50)
#             print("Motor running at 50% duty cycle anticlockwise")
#         else:
#             print('Invalid input. Please enter 0, 1, or -1.')
#             continue
#
#         print(f"Counter: {counter}")
#
# except KeyboardInterrupt:
#     print("\nProgram terminated.")
#     print("Final encoder count:", counter)
#     # Clean up GPIO pins
#     pwm.stop()
#     GPIO.cleanup()

import RPi.GPIO as GPIO
import gpiozero
import time
import threading

# Motor pins
DIR = 27
PWM = 22

# Encoder pins
CHANNEL_A_PIN = 23
CHANNEL_B_PIN = 24

# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Motor setup
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(PWM, GPIO.OUT)
GPIO.output(DIR, GPIO.HIGH)  # Set initial direction to clockwise
pwm = GPIO.PWM(PWM, 100)  # Initializing PWM at 100Hz
pwm.start(0)  # Start at 0% duty cycle / motor off

# Encoder setup
encoder_a = gpiozero.DigitalInputDevice(CHANNEL_A_PIN)
encoder_b = gpiozero.DigitalInputDevice(CHANNEL_B_PIN)

# Initialize the counter
counter = 0
print_lock = threading.Lock()

def encoder_counter():
    global counter
    if encoder_b.value:
        counter += 1
    else:
        counter -= 1

# Set up the encoder pins to trigger on rising edges
encoder_a.when_activated = encoder_counter

def print_counter():
    global counter
    while True:
        with print_lock:
            print(f"Encoder count: {counter}")
        time.sleep(1)

# Start the counter printing thread
counter_thread = threading.Thread(target=print_counter, daemon=True)
counter_thread.start()

print("Motor control and encoder counter started.")
print("Enter: 0 for no power, 1 for 50% clockwise, -1 for 50% anticlockwise")
print("Ctrl+C to exit")

try:
    while True:
        # Takes input
        user_input = int(input("Enter command (0, 1, or -1): "))

        if user_input == 0:
            pwm.ChangeDutyCycle(0)
            with print_lock:
                print("Motor stopped")
        elif user_input == 1:
            GPIO.output(DIR, GPIO.HIGH)  # Set direction to clockwise
            pwm.ChangeDutyCycle(50)
            with print_lock:
                print("Motor running at 50% duty cycle clockwise")
        elif user_input == -1:
            GPIO.output(DIR, GPIO.LOW)  # Set direction to anticlockwise
            pwm.ChangeDutyCycle(50)
            with print_lock:
                print("Motor running at 50% duty cycle anticlockwise")
        else:
            with print_lock:
                print('Invalid input. Please enter 0, 1, or -1.')
            continue

except KeyboardInterrupt:
    print("\nProgram terminated.")
    print("Final encoder count:", counter)
    # Clean up GPIO pins
    pwm.stop()
    GPIO.cleanup()