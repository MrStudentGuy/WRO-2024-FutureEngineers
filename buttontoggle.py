import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)


def led_on():
    GPIO.output(17, GPIO.HIGH)
    print("LED ON")


def led_off():
    GPIO.output(17, GPIO.LOW)
    print("LED OFF")


def toggle_led():
    ledstate = GPIO.input(17)
    if ledstate == 1:
        led_off()
    elif ledstate == 0:
        led_on()


if __name__ == '__main__':
    print("Press the button to toggle the light")
    print("Ctrl+C to exit")

    while True:
        buttonstate = GPIO.input(18)
        if buttonstate == 0:
            toggle_led()
            time.sleep(0.01)

    GPIO.cleanup()
