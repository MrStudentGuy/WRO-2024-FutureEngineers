import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def led_toggle():
    if GPIO.output(17) == GPIO.HIGH:
        GPIO.output(17, GPIO.LOW)
    else:
        GPIO.output(17, GPIO.HIGH)


if __name__ == '__main__':

    while True:
        if GPIO.input(18) == GPIO.HIGH:
            led_toggle()

    GPIO.cleanup()