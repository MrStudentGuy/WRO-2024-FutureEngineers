import RPi.GPIO as GPIO
import time

# Constants
CHANNEL_A_PIN = 23
CHANNEL_B_PIN = 27
BOUNCE_TIME = 10

# Set up GPIO pins for encoder channels A and B
GPIO.setmode(GPIO.BCM)

# Set GPIO mode to input explicitly
GPIO.setup(CHANNEL_A_PIN, GPIO.IN)
GPIO.setup(CHANNEL_B_PIN, GPIO.IN)

# Set pull-up resistors
GPIO.setup(CHANNEL_A_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(CHANNEL_B_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Initialize encoder counter
encoder_count = 0

# Define a function to handle encoder interrupts
def encoder_interrupt(channel):
    global encoder_count
    if channel == CHANNEL_A_PIN:  # Channel A
        if GPIO.input(CHANNEL_B_PIN):  # Channel B
            encoder_count += 1
        else:
            encoder_count -= 1
    elif channel == CHANNEL_B_PIN:  # Channel B
        if GPIO.input(CHANNEL_A_PIN):  # Channel A
            encoder_count -= 1
        else:
            encoder_count += 1

# Set up interrupts for encoder channels A and B
try:
    GPIO.add_event_detect(CHANNEL_A_PIN, GPIO.RISING, callback=encoder_interrupt, bouncetime=BOUNCE_TIME)
    GPIO.add_event_detect(CHANNEL_B_PIN, GPIO.RISING, callback=encoder_interrupt, bouncetime=BOUNCE_TIME)
except RuntimeError as e:
    print("Error adding edge detection:", e)
    exit(1)

try:
    while True:
        print("Encoder count:", encoder_count)
        time.sleep(0.1)

except KeyboardInterrupt:
    GPIO.cleanup()