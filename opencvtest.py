from picamera2 import Picamera2
import cv2

# Initialising and starting picamera2 object
cam = Picamera2()
cam.start()

if __name__ == '__main__':
    try:
        while True:
            frame = cam.capture_array() # Constantly capturing frames / video
            cv2.imshow('Video', frame) # Creating OpenCV preview using captured frames, named 'Video'
            cv2.waitKey(1) # Allows stream to continue until a key is pressed / program terminated
    except KeyboardInterrupt:
        # Stops camera feed when Ctrl + C is pressed
        frame = cam.capture_array()
        cam.stop()
        cv2.destroyAllWindows()