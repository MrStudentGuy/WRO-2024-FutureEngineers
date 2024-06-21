import multiprocessing
import queue
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

number = 0


def count1():
    for i in range(0, 10):
        number = i


def count2():
    for i in range (0, 10):
        print(number)


# Ensures it is run only as a script, not an import
if __name__ == '__main__':

    c1 = multiprocessing.Process(target=count1)
    c2 = multiprocessing.Process(target=count2)

    c1.start()
    c2.start()

    c1.join()  # Wait for process 1 to finish
    c2.join()  # Wait for process 2 to finish
