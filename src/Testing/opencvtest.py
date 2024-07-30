from picamera2 import Picamera2
import cv2

# Initialising and starting picamera2 object
cam = Picamera2()
cam.start()

if __name__ == '__main__':
    try:
        while True:
            frame = cam.capture_array() # Constantly capturing frames / video

            # Converting image to grayscale for cv2.Canny to better detect edges
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 30, 200)

            # Contours
            contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) # Draws contours from Canny file
            print(len(contours)) # Prints number of contours seen in each frame for debugging
            cv2.drawContours(frame, contours, -1, (255, 255, 255), 3) # Draws contours on frame in white (rgb 255,255,255)

            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            cv2.imshow('Video', frame) # Creating OpenCV preview of captured frame named 'Video'
            cv2.waitKey(1) # Allows stream to continue until a key is pressed / program terminated
    except KeyboardInterrupt:
        # Stops camera feed when Ctrl + C is pressed
        cam.stop()
        cv2.destroyAllWindows()