import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17, GPIO.OUT)

def led_on():
    GPIO.output(17, GPIO.HIGH)

def led_off():
    GPIO.output(17, GPIO.LOW)


if __name__ == '__main__':
    print("Enter 1 to turn on the light, 0 to turn off the light")
    print("Ctrl+C to exit")

    while True:
        lightstate = int(input())
        match lightstate:
            case 1:
                led_on()
            case 0:
                led_off()
            case 2:
                break
    GPIO.cleanup()
