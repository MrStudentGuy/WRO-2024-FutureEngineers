from picamera2 import Picamera2
import cv2

# Initialising and starting picamera2 object
cam = Picamera2()
cam.start()

cv2.cvtColor(cv2.COLOR_BGR2RGB)

if __name__ == '__main__':
    try:
        while True:
            frame = cam.capture_array() # Constantly capturing frames / video
            frameUMat = np.float32(frame)
            rgb_frame = cv2.cvtColor(frameUMat, cv2.COLOR_BGR2RGB)
            cv2.imshow('Video', rgb_frame) # Creating OpenCV preview using captured frames, named 'Video'
            cv2.waitKey(1) # Allows stream to continue until a key is pressed / program terminated
    except KeyboardInterrupt:
        # Stops camera feed when Ctrl + C is pressed
        rgb_frame = cam.capture_array()
        cam.stop()
        cv2.destroyAllWindows()