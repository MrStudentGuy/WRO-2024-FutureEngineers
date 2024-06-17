import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Initialising pins
LED = 17

GPIO.setup(LED, GPIO.OUT)

# Turns LED on
def led_on():
    GPIO.output(LED, GPIO.HIGH)

# Turns LED off
def led_off():
    GPIO.output(LED, GPIO.LOW)

# Ensures it is run only as a script, not an import
if __name__ == '__main__':
    print("Enter 1 to turn on the light, 0 to turn off the light")
    print("Ctrl+C to exit")
    
    try:
        while True:
            lightstate = int(input())
            match lightstate:
                case 1:
                    led_on()
                case 0:
                    led_off()
                    
    except KeyboardInterrupt:
        # Cleans up GPIO pins / resets state when terminated using Ctrl + C
        GPIO.cleanup()
