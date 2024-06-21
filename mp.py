import multiprocessing
import queue
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

def process():
    for i in range(0, 10):
        number = i


def print(number):
    for i in range (0, 10):
        print(number)


# Ensures it is run only as a script, not an import
if __name__ == '__main__':

    number = multiprocessing.Value('i', 0)

    c1 = multiprocessing.Process(target=process, args=number)
    c2 = multiprocessing.Process(target=print, args=number)

    c1.start()
    c2.start()

    c1.join()  # Wait for process 1 to finish
    c2.join()  # Wait for process 2 to finish
