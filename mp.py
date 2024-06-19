import multiprocessing
import os
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


def count1():
    i = 0
    for i in range(10):
        print(f"1: {i}")
        time.sleep(1)


def count2():
    i = 10
    for i in range(20):
        print(f"2: {i}")
        time.sleep(1)


c1 = multiprocessing.Process(target = count1(), args=())
c2 = multiprocessing.Process(target=count2(), args=())

c1.start()
c2.start()