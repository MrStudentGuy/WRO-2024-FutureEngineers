import RPi.GPIO as GPIO
import time

# Set up GPIO pins for encoder channels A and B
GPIO.setmode(GPIO.BCM)
GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Channel A
GPIO.setup(23, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Channel B

# Initialize encoder counter
encoder_count = 0

# Define a function to handle encoder interrupts
def encoder_interrupt(channel):
    global encoder_count
    if channel == 27:  # Channel A
        if GPIO.input(23):  # Channel B
            encoder_count += 1
        else:
            encoder_count -= 1
    elif channel == 23:  # Channel B
        if GPIO.input(27):  # Channel A
            encoder_count -= 1
        else:
            encoder_count += 1

# Set up interrupts for encoder channels A and B
GPIO.add_event_detect(27, GPIO.RISING, callback=encoder_interrupt, bouncetime=10)
GPIO.add_event_detect(23, GPIO.RISING, callback=encoder_interrupt, bouncetime=10)

try:
    while True:
        print("Encoder count:", encoder_count)
        time.sleep(0.1)

except KeyboardInterrupt:
    GPIO.cleanup()