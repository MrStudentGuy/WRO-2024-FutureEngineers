import numpy as np
import cv2
from picamera2 import Picamera2
import json


class HSVColorTracker:
    def __init__(self):
        self.hue_min = 0
        self.hue_max = 179
        self.sat_min = 0
        self.sat_max = 255
        self.value_min = 0
        self.value_max = 255
        self.r1_min = 0
        self.r1_max = 179
        self.color_option = 'red'

    def save_hsv_bounds(self, filename='hsv_bounds.json'):
        bounds = {
            'HUE Min': self.hue_min,
            'HUE Max': self.hue_max,
            'SAT Min': self.sat_min,
            'SAT Max': self.sat_max,
            'VALUE Min': self.value_min,
            'VALUE Max': self.value_max,
            'R1 Min': self.r1_min,
            'R1 Max': self.r1_max
        }
        with open(filename, 'w') as f:
            json.dump(bounds, f)

    def load_hsv_bounds(self, filename='hsv_bounds.json'):
        try:
            with open(filename, 'r') as f:
                bounds = json.load(f)
                self.hue_min = bounds['HUE Min']
                self.hue_max = bounds['HUE Max']
                self.sat_min = bounds['SAT Min']
                self.sat_max = bounds['SAT Max']
                self.value_min = bounds['VALUE Min']
                self.value_max = bounds['VALUE Max']
                self.r1_min = bounds['R1 Min']
                self.r1_max = bounds['R1 Max']
        except FileNotFoundError:
            print("No previous HSV bounds found.")


def empty(a):
    pass


def resize_final_img(x, y, *argv):
    images = cv2.resize(argv[0], (x, y))
    for i in argv[1:]:
        resize = cv2.resize(i, (x, y))
        images = np.concatenate((images, resize), axis=1)
    return images


if __name__ == "__main__":
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (1600, 1000)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.preview_configuration.align()
    picam2.configure("preview")
    picam2.start()

    tracker = HSVColorTracker()
    tracker.load_hsv_bounds()

    cv2.namedWindow("HSV")
    cv2.resizeWindow("HSV", 300, 300)

    # Trackbars for HSV values
    cv2.createTrackbar("HUE Min", "HSV", tracker.hue_min, 179, empty)
    cv2.createTrackbar("HUE Max", "HSV", tracker.hue_max, 179, empty)
    cv2.createTrackbar("SAT Min", "HSV", tracker.sat_min, 255, empty)
    cv2.createTrackbar("SAT Max", "HSV", tracker.sat_max, 255, empty)
    cv2.createTrackbar("VALUE Min", "HSV", tracker.value_min, 255, empty)
    cv2.createTrackbar("VALUE Max", "HSV", tracker.value_max, 255, empty)
    cv2.createTrackbar("R1 Min", "HSV", tracker.r1_min, 179, empty)
    cv2.createTrackbar("R1 Max", "HSV", tracker.r1_max, 179, empty)

    # Radio buttons for color selection
    color_options = ["Red", "Green", "Pink", "White"]
    cv2.createTrackbar("Color", "HSV", 0, len(color_options) - 1, empty)

    cv2.resizeWindow('F', 700, 600)

    while True:
        img = picam2.capture_array()

        # Update HSV bounds from trackbars
        tracker.hue_min = cv2.getTrackbarPos("HUE Min", "HSV")
        tracker.hue_max = cv2.getTrackbarPos("HUE Max", "HSV")
        tracker.sat_min = cv2.getTrackbarPos("SAT Min", "HSV")
        tracker.sat_max = cv2.getTrackbarPos("SAT Max", "HSV")
        tracker.value_min = cv2.getTrackbarPos("VALUE Min", "HSV")
        tracker.value_max = cv2.getTrackbarPos("VALUE Max", "HSV")
        tracker.r1_min = cv2.getTrackbarPos("R1 Min", "HSV")
        tracker.r1_max = cv2.getTrackbarPos("R1 Max", "HSV")

        # Get selected color option
        color_option_index = cv2.getTrackbarPos("Color", "HSV")
        tracker.color_option = color_options[color_option_index]

        # Convert to HSV
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        lower = np.array([tracker.hue_min, tracker.sat_min, tracker.value_min])
        upper = np.array([tracker.hue_max, tracker.sat_max, tracker.value_max])
        r1upper = np.array([tracker.r1_max, tracker.sat_max, tracker.value_max])
        r1lower = np.array([tracker.r1_min, tracker.sat_min, tracker.value_min])
        mask1 = cv2.inRange(hsv_img, lower, upper)
        mask2 = cv2.inRange(hsv_img, r1lower, r1upper)

        # Adjust mask based on selected color option
        if tracker.color_option in ["Red", "White"]:
            mask = mask1 + mask2
        else:
            mask = mask1  # For Green and Pink, use only mask1

        kernel = np.ones((3, 3), 'uint8')
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)
        d_img = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        # Final resized image for display
        final_img = resize_final_img(300, 300, d_img)
        cv2.imshow('F', final_img)

        # Check for save button
        if cv2.waitKey(1) & 0xFF == ord('s'):  # Press 's' to save
            tracker.save_hsv_bounds()
            print("HSV bounds saved.")

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cv2.destroyAllWindows()
