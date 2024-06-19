import multiprocessing
import os
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


def count1():
    i = 0
    for i in range(10):
        print(i)
        time.sleep(1)


def count2():
    i = 10
    for i in range(10):
        print(i)
        time.sleep(1)


# Ensures it is run only as a script, not an import
if __name__ == '__main__':

    try:
        while True:

            c1 = multiprocessing.Process(target = count1(), args=())
            c2 = multiprocessing.Process(target=count2(), args=())

            c1.start()
            c2.start()

    except KeyboardInterrupt:
        # Cleans up GPIO pins / resets state when terminated using Ctrl + C
        GPIO.cleanup()