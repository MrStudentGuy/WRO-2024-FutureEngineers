import multiprocessing
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


def calc(numb):
    i = 0
    while i < 10:
        numb.value = numb.value + 1
        i = i + 1


def output(numb):
    print(numb.value)


# Ensures it is run only as a script, not an import
if __name__ == '__main__':
    numb = multiprocessing.Value('i', 0)

    c1 = multiprocessing.Process(target=calc, args=(numb,))
    c2 = multiprocessing.Process(target=output, args=(numb, ))

    c1.start()
    c2.start()
