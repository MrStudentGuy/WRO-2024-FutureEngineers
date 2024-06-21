import multiprocessing
import queue
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


def calc(numb):
    for i in range (0,10):
        numb.value = numb.value + 1


def output(numb):
    for i in range (0,10):
        print(numb.value)


# Ensures it is run only as a script, not an import
if __name__ == '__main__':
    numb = multiprocessing.Value('i', 0)

    c1 = multiprocessing.Process(target=calc, args=(numb,))
    c2 = multiprocessing.Process(target=output, args=(numb, ))

    c1.start()
    c2.start()

    c1.join()  # Wait for process 1 to finish
    c2.join()  # Wait for process 2 to finish
