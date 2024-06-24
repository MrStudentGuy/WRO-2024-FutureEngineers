import time
from picamera2 import Picamera2, MappedArray
import cv2

# Initialising picamera2 object
picam2 = Picamera2()

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
picam2.pre_callback = apply_timestamp
picam2.start(show_preview=True)

# Delay till function end / camera running time
time.sleep(5)