from picamera2 import Picamera2
import cv2
import numpy as np

# Initialising and starting picamera2 object
cam = Picamera2()
cam.start()

if __name__ == '__main__':
    try:
        while True:
            # Capturing frame
            frame = cam.capture_array()

            # Converting to RGB for viewing purposes
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # Converting to HSV
            frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Setting HSV colour bounds for green and red
            lower_green = np.array([40, 88, 38])
            upper_green = np.array([59, 163, 122])

            lower_red = np.array([155, 99, 56])
            upper_red = np.array([179, 190, 149])

            # Using bounds to create masks for green and red objects
            mask_green = cv2.inRange(frame_hsv, lower_green, upper_green)
            mask_red = cv2.inRange(frame_hsv, lower_red, upper_red)

            # Combining masks
            mask = cv2.bitwise_or(mask_green, mask_red)

            # Converting image to grayscale for cv2.Canny to better detect edges
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 30, 200)

            # Applying mask to edges
            edges_masked = cv2.bitwise_and(edges, edges, mask=mask)

            # Contours
            contours, hierarchy = cv2.findContours(edges_masked, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE) # Draws contours from Canny file
            print(len(contours)) # Prints number of contours seen in each frame for debugging
            cv2.drawContours(frame, contours, -1, (255, 255, 255), 3) # Draws contours on frame in white (rgb 255,255,255)

            cv2.imshow('Video', frame_rgb) # Creating OpenCV preview of captured frame named 'Video'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()
    except KeyboardInterrupt:
        # Stops camera feed when Ctrl + C is pressed
        cam.stop()
        cv2.destroyAllWindows()