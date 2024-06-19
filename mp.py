import multiprocessing
import queue
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


def count1(q):
    for i in range(0, 10):
        time.sleep(1)
        q.put(f"1: {i}")


def count2():
    while True:
        print(q.get())


# Ensures it is run only as a script, not an import
if __name__ == '__main__':

    q = queue.Queue()

    c1 = multiprocessing.Process(target=count1, args=(q,))
    c2 = multiprocessing.Process(target=count2, args=(q,))

    c1.start()
    c2.start()

    c1.join()  # Wait for process 1 to finish
    c2.join()  # Wait for process 2 to finish
