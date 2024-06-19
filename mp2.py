import multiprocessing
import time
from multiprocessing import Pipe

def count1(conn):
    for i in range(0, 10):
        print(f"1: {i}")
        time.sleep(1)
        conn.send(f"Count 1: {i}")  # Send message through pipe

def count2(conn):
    while True:
        data = conn.recv()  # Receive message from pipe
        if data == 'done':
            break
        print(f"Received from 1: {data}")
        time.sleep(1)

if __name__ == "__main__":
    parent_conn, child_conn = Pipe()

    c1 = multiprocessing.Process(target=count1, args=(child_conn,))
    c2 = multiprocessing.Process(target=count2, args=(parent_conn,))

    c1.start()
    c2.start()

    c1.join()  # Wait for process 1 to finish
    child_conn.send('done')  # Signal process 2 to finish
    c2.join()  # Wait for process 2 to finish

    print("Both processes finished!")