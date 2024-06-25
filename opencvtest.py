from picamera2 import Picamera2
import cv2

# Initialising and starting picamera2 object
cam = Picamera2()
cam.start()

if __name__ == '__main__':
    try:
        while True:
            frame = cam.capture_array() # Constantly capturing frames / video
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Converts each frame to HSV colour
            edges = cv2.Canny(frame, 30, 200)
            contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            print(len(contours)) # Prints number of contours seen in each frame for debugging
            cv2.drawContours(frame, contours, -1, (255, 255, 255), 3) # Draws contours on frame in white (rgb 255,255,255)
            cv2.imshow('Video', frame) # Creating OpenCV preview using captured frames, named 'Video'
            cv2.waitKey(1) # Allows stream to continue until a key is pressed / program terminated
    except KeyboardInterrupt:
        # Stops camera feed when Ctrl + C is pressed
        cam.stop()
        cv2.destroyAllWindows()