from picamera2 import Picamera2
import cv2
import numpy as np

# Initialising and starting picamera2 object
cam = Picamera2()
cam.start()

# General variables needed for OpenCV
kernel = np.ones((3,3),'uint8')

# Flags for colour presence
green_present = False
red_present = False


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
            lower_green = np.array([40, 88, 30])
            upper_green = np.array([59, 163, 122])

            lower_red = np.array([0, 100, 100])
            upper_red = np.array([10, 255, 255])

            # Using bounds to create masks for green and red objects
            mask_green = cv2.inRange(frame_hsv, lower_green, upper_green)
            mask_red = cv2.inRange(frame_hsv, lower_red, upper_red)

            # Functions to increase detection accuracy of masks
            mask_green = cv2.dilate(mask_green, kernel, iterations=3)
            mask_green = cv2.erode(mask_green, kernel, iterations=3)
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel,iterations = 5)

            mask_red = cv2.dilate(mask_red, kernel, iterations=3)
            mask_red = cv2.erode(mask_red, kernel, iterations=3)
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel, iterations=5)

            # Finding and sorting contours for either colour
            contours_green, hierarchy_green = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # contours_green = sorted(contours_green, key=cv2.contourArea, reverse=True)[:1]

            contours_red, hierarchy_red = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # contours_red = sorted(contours_red, key=cv2.contourArea, reverse=True)[:1]

            # # Taking maximum count of contours to highlight (removing small colour anomalies)
            # if len(contours_green) == 0:
            #     print("Cant find contour for green....")
            #     green_present = False
            # else:
            #
            #     max_cnt_green = max(contours_green, key=cv2.contourArea)
            #     green_present = True
            #
            # if len(contours_red) == 0:
            #     print("Cant find contour for red....")
            #     red_present = False
            # else:
            #
            #     max_cnt_red = max(contours_red, key=cv2.contourArea)
            #     red_present = True
            cv2.drawContours(frame_rgb, contours_green, -1, (255, 255, 255), 3)
            cv2.drawContours(frame_rgb, contours_red, -1, (255, 255, 255), 3)

            cv2.imshow('Video', frame_rgb) # Creating OpenCV preview of captured frame named 'Video'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cv2.destroyAllWindows()
    except KeyboardInterrupt:
        # Stops camera feed when Ctrl + C is pressed
        cam.stop()
        cv2.destroyAllWindows()