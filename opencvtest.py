import time
from picamera2 import Picamera2, MappedArray
import cv2

# Initialising and starting picamera2 object
cam = Picamera2()
cam.start()

while True:
    frame = cam.capture_array() # Constantly capturing frames / video
    cv2.imshow('Video', frame) # Creating OpenCV preview using captured frames, named 'Video'