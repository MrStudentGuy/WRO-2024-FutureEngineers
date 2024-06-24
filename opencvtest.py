import time
from picamera2 import Picamera2, MappedArray
import cv2

# Initialising picamera2 object
cam = Picamera2()

# Timestamp variables
colour = (0, 255, 0)
origin = (0, 30)
font = cv2.FONT_HERSHEY_SIMPLEX
scale = 1
thickness = 2

# Timestamp function
def apply_timestamp(request):
  timestamp = time.strftime("%Y-%m-%d %X")
  with MappedArray(request, "main") as m:
    cv2.putText(m.array, timestamp, origin, font, scale, colour, thickness)

# Running camera
cam.pre_callback = apply_timestamp
cam.start(show_preview=True)

while True:
    frame = cam.capture_array()
    cv2.imshow('Video', frame)
    cv2.waitKey(1)