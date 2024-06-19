import multiprocessing
import queue
import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)


def count1(q):
    for i in range(0, 10):
        print(f"1: {i}")
        time.sleep(1)
        q.put(f"2: {i}")


def count2(q):
    while True:
        data = q.get()  # Receive message from queue
        if data == 'done':
            break
        print(f"Received from 1: {data}")
        time.sleep(1)


# Ensures it is run only as a script, not an import
if __name__ == '__main__':

    q = queue.Queue()

    c1 = multiprocessing.Process(target=count1, args=(q,))
    c2 = multiprocessing.Process(target=count2, args=(q,))

    c1.start()
    c2.start()

    c1.join()  # Wait for process 1 to finish
    q.put('done')
    c2.join()  # Wait for process 2 to finish
