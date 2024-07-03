from picamera2 import Picamera2
import cv2
import numpy as np

# Initialize the camera
picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"format": 'XRGB8888', "size": (640, 480)}))
picam2.start()

while True:
    # Capture frame-by-frame
    frame = picam2.capture_array()

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range of green color in HSV
    lower_green = np.array([40, 40, 40])
    upper_green = np.array([80, 255, 255])

    # Define range of red color in HSV
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([160, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    # Threshold the HSV image to get green and red colors
    mask_green = cv2.inRange(hsv, lower_green, upper_green)
    mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask_red = cv2.bitwise_or(mask_red1, mask_red2)

    # Combine masks
    mask = cv2.bitwise_or(mask_green, mask_red)

    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Find the largest contour
    largest_contour = max(contours, key=cv2.contourArea, default=None)

    if largest_contour is not None and cv2.contourArea(largest_contour) > 100:
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Determine the color of the largest contour
        roi = hsv[y:y+h, x:x+w]
        avg_color = np.mean(roi, axis=(0, 1))

        if 40 <= avg_color[0] <= 80:
            color = (0, 255, 0)  # Green
            label = "Green"
        else:
            color = (0, 0, 255)  # Red
            label = "Red"

        cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
        cv2.putText(frame, label, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, color, 2)

    # Display the resulting frame
    cv2.imshow('Color Detection', frame)

    # Break the loop on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
picam2.stop()
cv2.destroyAllWindows()