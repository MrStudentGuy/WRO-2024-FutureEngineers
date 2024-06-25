from picamera2 import Picamera2
import cv2

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
            lower_green = (40, 40, 40)
            upper_green = (80, 255, 255)

            lower_red = (0, 40, 40)
            upper_red = (10, 255, 255)

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
            if cv2.waitKey(1):
                break

    except KeyboardInterrupt:
        # Stops camera feed when Ctrl + C is pressed
        cam.stop()
        cv2.destroyAllWindows()