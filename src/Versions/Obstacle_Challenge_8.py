import os

os.system('sudo pkill pigpiod')
os.system('sudo pigpiod')
import sys
import logging

import numpy as np
import RPi.GPIO as GPIO
import cv2
from picamera2 import Picamera2
import time

import multiprocessing
import pigpio
import board

import math
from imports.Encoder import EncoderCounter
from imports.BNO085 import IMUandColorSensor
from imports.Servo import Servo
import serial
import RPi.GPIO as GPIO
from imports.TFmini import TFmini

log_file = open('/home/pi/WRO-2024-FutureEngineers/Logs/log_obstacle.txt', 'w')
sys.stdout = log_file
time.sleep(5)

GPIO.setmode(GPIO.BCM)
GPIO.setup(8, GPIO.OUT)
GPIO.setup(26, GPIO.OUT)  # blue
GPIO.setup(10, GPIO.OUT)  # red
GPIO.setup(6, GPIO.OUT)  # green
GPIO.output(26, GPIO.LOW)
GPIO.output(6, GPIO.LOW)
GPIO.output(10, GPIO.LOW)

print("Resetting....")
GPIO.output(8, GPIO.LOW)
GPIO.output(6, GPIO.HIGH)
time.sleep(1)
GPIO.output(8, GPIO.HIGH)
GPIO.output(6, GPIO.LOW)

time.sleep(1)

print("Reset Complete")

imu = IMUandColorSensor(board.SCL, board.SDA)
tfmini = TFmini(23, 24, 25)
GPIO.setwarnings(False)

GPIO.setmode(GPIO.BCM)

GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Button to GPIO23

# pwm = pigpio.pi()
# Parameters for servo
servo = Servo(8)

RX_Head = 23
RX_Left = 24
RX_Right = 25
# pi = pigpio.pi()

# Define object specific variables for gree

currentAngle = 0
error_gyro = 0
prevErrorGyro = 0
totalErrorGyro = 0
correcion = 0
totalError = 0
prevError = 0
kp = 0.6
ki = 0
kd = 0.1

kp_e = 3  # 12
ki_e = 0
kd_e = 40  # 40if

setPoint_flag = 0

corr = 0
corr_pos = 0


def correctPosition(setPoint, head, x, y, counter, blue, orange, reset, reverse, heading, centr_x, finish):
    # print("INSIDE CORRECT")
    # getTFminiData()
    global prevError, totalError, prevErrorGyro, totalErrorGyro, corr_pos

    error = 0
    correction = 0
    pTerm_e = 0
    dTerm_e = 0
    iTerm_e = 0
    lane = counter % 4
    # if(time.time() - last_time > 0.001):
    if lane == 0:
        if not reverse:
            if orange:
                error = setPoint - y
            else:
                error = setPoint - y
            print(f" 1 lane: {lane}, error: {error} target:{(setPoint)}, x:{x} y:{y} not reverse")

        elif reverse:
            if blue:
                error = setPoint - y
            elif orange:
                error = setPoint - y
            print(f"2 lane: {lane}, error: {error} target:{(setPoint)}, x:{x} y:{y} not reverse")

    # print(f"lane:{lane} error: {error} tagret:{setPoint}")
    # print(f"trigger : {flag_t} setPoint: {setPoint} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
    elif lane == 1:
        if orange:
            error = x - (100 - setPoint)
            print(f"lane:{lane}, error:{error} target:{(100 - setPoint)}, x:{x}, y:{y}")

        elif blue:
            error = (100 + setPoint) - x
            print(f"lane:{lane}, error:{error} target:{(100 + setPoint)}, x:{x} y:{y} Bluee")
    # print(f" trigger : {flag_t} setPoint: {setPoint} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
    elif lane == 2:
        if orange:
            error = y - (200 - setPoint)
            print(f"lane:{lane} error:{error} target:{(200 - setPoint)},  x: {x} y{y}")
        elif blue:
            error = y - (-200 - setPoint)
            print(f"lane:{lane} error:{error} target:{(-200 - setPoint)}, x: {x} y{y}")
    # print(f"setPoint: {flag_t} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
    elif lane == 3:
        if orange:
            error = (setPoint - 100) - x
            print(f"lane:{lane} error:{error} target:{(setPoint - 100)}, x: {x} y {y}")

        elif blue:
            error = x + (100 + setPoint)
            print(f"lane:{lane} error:{error} target:{(100 + setPoint)}, x:{x} y {y}")

    # print(f"lane:{lane} error: {error} target:{-100 - setPoint}")
    # print(f"setPoint: {flag_t} lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
    # last_time = time.time()

    corr_pos = error
    pTerm_e = kp_e * error
    dTerm_e = kd_e * (error - prevError)
    totalError += error
    iTerm_e = ki_e * totalError
    correction = pTerm_e + iTerm_e + dTerm_e

    # print("correction: {}, x:{}, y:{}, heading:{} ".format(correction, x, y, glob))

    # if (setPoint_flag == 0)

    if setPoint == 0:
        if abs(error) < 10:
            print("absolute is 0")
            correction = 0

    # print("In the  correct Position")

    if not reset:
        tfmini.getTFminiData()

        if not blue:
            if (setPoint <= -70) and tfmini.distance_left <= 25:
                print(f"Correcting Green Wall Orange")
                correction = 10
            else:
                pass

            if setPoint >= 70 and (tfmini.distance_right < 25 or (tfmini.distance_head < 15)):
                print(f"Wall detected..making correction")
                # if red == 0:
                correction = -10
            else:
                pass

        else:
            if setPoint <= -70 and (tfmini.distance_left <= 25 or (tfmini.distance_head < 15)):
                print(f"Wall detected..making correction")
                # if green == 0:
                correction = 10
            else:
                pass
            if setPoint >= 70 and tfmini.distance_right < 25:
                print(f"correctng red wall in blue")
                correction = -10
            else:
                pass

    if setPoint == 0:
        if correction > 25:
            correction = 25
        elif correction < -25:
            correction = -25
    else:
        if correction > 45:
            correction = 45
        elif correction < -45:
            correction = -45

    # print(f"Correction in position:{correction}")

    # print(f"lane: {lane} correction:{correction}, error:{error} x:{x}, y:{y}, prevError :{prevError} angle:{head - correction}")
    # print("correction: ", correction)

    prevError = error
    # print(f"Correction: {head - correction}")
    correctAngle(head + correction, heading)


def correctAngle(setPoint_gyro, heading):
    # print("INSIDE CORRECT")
    # time.sleep(0.001)
    global corr

    error_gyro = 0
    prevErrorGyro = 0
    totalErrorGyro = 0
    correction = 0
    totalError = 0
    prevError = 0

    error_gyro = heading - setPoint_gyro

    if error_gyro > 180:
        error_gyro = error_gyro - 360
    corr = error_gyro
    # print("Error : ", error_gyro)
    pTerm = 0
    dTerm = 0
    iTerm = 0

    pTerm = kp * error_gyro
    dTerm = kd * (error_gyro - prevErrorGyro)
    totalErrorGyro += error_gyro
    iTerm = ki * totalErrorGyro
    correction = pTerm + iTerm + dTerm

    if correction > 30:
        correction = 30
    elif correction < -30:
        correction = -30

    prevErrorGyro = error_gyro
    servo.setAngle(90 - correction)


# Extract Frames

# basic constants for opencv Functs
kernel = np.ones((3, 3), 'uint8')
font = cv2.FONT_HERSHEY_SIMPLEX
org = (0, 20)
fontScale = 0.6
color = (0, 0, 255)
thickness = 2


# loop to capture video frames
def Live_Feed(color_b, stop_b, red_b, green_b, pink_b, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink,
              centr_y_pink):
    print('Image Process started')
    both_flag = False
    all_flag = False
    only_red = False
    only_green = False
    only_pink = False
    pink_red = False
    pink_green = False
    while True:
        try:
            picam2 = Picamera2()
            break
        except RuntimeError:
            picam2.uninit()
            picam2 = Picamera2()
    picam2.preview_configuration.main.size = (1600, 1000)  # (1300, 800)
    picam2.preview_configuration.main.format = 'RGB888'

    picam2.preview_configuration.align()
    picam2.configure('preview')

    picam2.start()

    cv2.namedWindow('Object Dist Measure ', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Object Dist Measure ', 1280, 720)

    while True:
        img = picam2.capture_array()

        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # green
        hsv_img1 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # red
        hsv_img2 = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # pink

        # predefined mask for green colour detection
        # For Green Color
        lower = np.array([45, 53, 44])  # green
        upper = np.array([76, 111, 136])
        mask = cv2.inRange(hsv_img, lower, upper)
        mask = cv2.dilate(mask, kernel, iterations=5)
        mask = cv2.erode(mask, kernel, iterations=5)

        # For Red Color
        lower1 = np.array([173, 76, 74])  # red
        upper1 = np.array([179, 181, 187])
        mask1 = cv2.inRange(hsv_img1, lower1, upper1)
        mask1 = cv2.dilate(mask1, kernel, iterations=5)
        mask1 = cv2.erode(mask1, kernel, iterations=5)

        # For Pink Color
        lower2 = np.array([164, 101, 69])  # pink
        upper2 = np.array([169, 197, 112])
        mask2 = cv2.inRange(hsv_img2, lower2, upper2)
        mask2 = cv2.dilate(mask2, kernel, iterations=5)
        mask2 = cv2.erode(mask2, kernel, iterations=5)

        # Remove Extra garbage from image
        d_img = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)  # green
        d_img1 = cv2.morphologyEx(mask1, cv2.MORPH_OPEN, kernel, iterations=1)  # red
        d_img2 = cv2.morphologyEx(mask2, cv2.MORPH_OPEN, kernel, iterations=1)  # pink

        # find the histogram
        cont, hei = cv2.findContours(d_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cont = sorted(cont, key=cv2.contourArea, reverse=True)[:1]

        cont1, hei1 = cv2.findContours(d_img1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cont1 = sorted(cont1, key=cv2.contourArea, reverse=True)[:1]

        cont2, hei2 = cv2.findContours(d_img2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cont2 = sorted(cont2, key=cv2.contourArea, reverse=True)[:1]

        # ----------------------------------------------
        if len(cont) == 0:
            green_present = False
        else:
            max_cnt = max(cont, key=cv2.contourArea)
            if cv2.contourArea(max_cnt) > 2000:
                green_present = True
            else:
                green_present = False

        # ---------------------------------------
        if len(cont1) == 0:
            red_present = False

        else:
            max_cnt1 = max(cont1, key=cv2.contourArea)
            if cv2.contourArea(max_cnt1) > 2000:
                red_present = True
            else:
                red_present = False
        # ----------------------------------------------------
        if len(cont2) == 0:
            pink_present = False

        else:
            max_cnt2 = max(cont2, key=cv2.contourArea)
            if cv2.contourArea(max_cnt2) > 2000:
                pink_present = True
            else:
                pink_present = False
        # --------------------------------------------------------

        if not red_present and not green_present and not pink_present:
            color_b.value = False
            red_b.value = False
            green_b.value = False
            pink_b.value = False
            all_flag = False
            both_flag = False
            pink_red = False
            pink_green = False
            only_red = False
            only_green = False
            only_pink = False

        if (red_present and green_present and pink_present):
            all_flag = True
            both_flag = False
            pink_red = False
            pink_green = False
            only_red = False
            only_green = False
            only_pink = False
        elif (red_present and green_present) and not pink_present:
            both_flag = True
            all_flag = False
            only_red = False
            only_green = False
            only_pink = False
            pink_red = False
            pink_green = False
        elif red_present and (not pink_present and not green_present):
            only_red = True
            both_flag = False
            all_flag = False
            only_green = False
            only_pink = False
            pink_red = False
            pink_green = False
        elif green_present and (not pink_present and not red_present):
            only_green = True
            both_flag = False
            all_flag = False
            only_red = False
            only_pink = False
            pink_red = False
            pink_green = False
        elif pink_present and (not green_present and not red_present):
            only_pink = True
            both_flag = False
            all_flag = False
            only_red = False
            only_green = False
            pink_red = False
            pink_green = False
        elif (pink_present and green_present) and not red_present:
            only_pink = False
            both_flag = False
            all_flag = False
            only_red = False
            only_green = False
            pink_red = False
            pink_green = True

        elif (pink_present and red_present) and not green_present:
            only_pink = False
            both_flag = False
            all_flag = False
            only_red = False
            only_green = False
            pink_red = True
            pink_green = False
        if all_flag:
            color_b.value = True
            # print("ITS THE FIRST LOOP")
            ### FOR GREEN BOX
            if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt1):
                if cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000:
                    # Draw a rectange on the contour
                    rect = cv2.minAreaRect(max_cnt)
                    box = cv2.boxPoints(rect)
                    box = np.intp(box)
                    cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
                    (x, y, w, h) = cv2.boundingRect(box)
                    centroid_y = y + h // 2
                    centroid_x = x + w // 2
                    centr_y.value = centroid_y
                    centr_x.value = centroid_x
                    centr_y_red.value = 0
                    centr_x_red.value = 0
                    green_b.value = True
                    red_b.value = False



            ### FOR RED BOX
            elif cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt):
                if (cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000):
                    # Draw a rectange on the contour
                    rect1 = cv2.minAreaRect(max_cnt1)
                    box = cv2.boxPoints(rect1)
                    box = np.intp(box)
                    cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

                    (x, y, w, h) = cv2.boundingRect(box)

                    centroid_y_red = y + h // 2
                    centroid_x_red = x + w // 2

                    centr_y_red.value = centroid_y_red
                    centr_x_red.value = centroid_x_red
                    centr_y.value = 0
                    centr_x.value = 0
                    red_b.value = True
                    green_b.value = False

            #### FOR PINK BOX
            # elif cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt) and cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt1):
            if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
                # Draw a rectange on the contour
                rect2 = cv2.minAreaRect(max_cnt2)
                box = cv2.boxPoints(rect2)
                box = np.intp(box)
                cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

                (x, y, w, h) = cv2.boundingRect(box)

                centroid_x = x + w // 2
                centroid_y = y + h // 2
                centr_x_pink.value = centroid_x
                centr_y_pink.value = centroid_y
                # if centroid_y > 500:
                pink_b.value = True


        ### FOR RED BOX
        elif only_red:
            color_b.value = True
            # print(cv2.contourArea(max_cnt1))
            if cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000:
                # Draw a rectange on the contour
                rect1 = cv2.minAreaRect(max_cnt1)
                box = cv2.boxPoints(rect1)
                box = np.intp(box)
                cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
                (x, y, w, h) = cv2.boundingRect(box)

                centroid_y_red = y + h // 2
                centroid_x_red = x + w // 2

                centr_y_red.value = centroid_y_red
                centr_x_red.value = centroid_x_red
                centr_y.value = 0
                centr_x.value = 0
                centr_x_pink.value = 0
                centr_y_pink.value = 0

                red_b.value = True
                pink_b.value = False
                green_b.value = False



        ### FOR GREEN BOX
        elif only_green:
            color_b.value = True
            # print(cv2.contourArea(max_cnt))
            if cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000:
                # Draw a rectange on the contour
                rect = cv2.minAreaRect(max_cnt)
                box = cv2.boxPoints(rect)
                box = np.intp(box)
                cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
                (x, y, w, h) = cv2.boundingRect(box)

                centroid_y = y + h // 2
                centroid_x = x + w // 2
                centr_y.value = centroid_y
                centr_x.value = centroid_x
                centr_y_red.value = 0
                centr_x_red.value = 0
                centr_x_pink.value = 0
                centr_y_pink.value = 0
                # if(centroid_y > 100):
                # if(counter_green >= max_count):
                green_b.value = True
                pink_b.value = False
                red_b.value = False

        ### FOR PINK BOX
        elif only_pink:
            if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
                # Draw a rectange on the contour
                rect2 = cv2.minAreaRect(max_cnt2)
                box = cv2.boxPoints(rect2)
                box = np.intp(box)
                cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

                (x, y, w, h) = cv2.boundingRect(box)

                centroid_x = x + w // 2
                centroid_y = y + h // 2
                centr_x_pink.value = centroid_x
                centr_y_pink.value = centroid_y
                centr_y_red.value = 0
                centr_x_red.value = 0
                centr_x.value = 0
                centr_y.value = 0
                red_b.value = False
                green_b.value = False
                pink_b.value = True

        elif both_flag:
            color_b.value = True
            # print("BOTH ARE PRESENT...")
            ### FOR GREEN BOX
            if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt1):
                if cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000:
                    # Draw a rectange on the contour
                    rect = cv2.minAreaRect(max_cnt)
                    box = cv2.boxPoints(rect)
                    box = np.intp(box)
                    cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
                    (x, y, w, h) = cv2.boundingRect(box)
                    centroid_y = y + h // 2
                    centroid_x = x + w // 2
                    centr_y.value = centroid_y
                    centr_x.value = centroid_x
                    centr_x_red.value = 0
                    centr_y_red.value = 0
                    centr_x_pink.value = 0
                    centr_y_pink.value = 0
                    green_b.value = True

                    red_b.value = False
                    pink_b.value = False

            ### FOR RED BOX
            elif cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt):
                if (cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000):
                    # Draw a rectange on the contour
                    rect1 = cv2.minAreaRect(max_cnt1)
                    box = cv2.boxPoints(rect1)
                    box = np.intp(box)
                    cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

                    (x, y, w, h) = cv2.boundingRect(box)

                    centroid_y_red = y + h // 2
                    centroid_x_red = x + w // 2

                    centr_y_red.value = centroid_y_red
                    centr_x_red.value = centroid_x_red
                    centr_x.value = 0
                    centr_y.value = 0
                    centr_x_pink.value = 0
                    centr_y_pink.value = 0
                    red_b.value = True
                    green_b.value = False
                    pink_b.value = False

        elif pink_red:
            ### FOR RED BOX
            # if cv2.contourArea(max_cnt1) > cv2.contourArea(max_cnt2):
            if (cv2.contourArea(max_cnt1) > 1000 and cv2.contourArea(max_cnt1) < 306000):
                # Draw a rectange on the contour
                rect1 = cv2.minAreaRect(max_cnt1)
                box = cv2.boxPoints(rect1)
                box = np.intp(box)
                cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

                (x, y, w, h) = cv2.boundingRect(box)

                centroid_y_red = y + h // 2
                centroid_x_red = x + w // 2

                centr_y_red.value = centroid_y_red
                centr_x_red.value = centroid_x_red
                centr_x.value = 0
                centr_y.value = 0
                red_b.value = True
                green_b.value = False  # pink_b.value = False
            # elif cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt1):
            if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
                # Draw a rectange on the contour
                rect2 = cv2.minAreaRect(max_cnt2)
                box = cv2.boxPoints(rect2)
                box = np.intp(box)
                cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

                (x, y, w, h) = cv2.boundingRect(box)

                centroid_x = x + w // 2
                centroid_y = y + h // 2
                centr_x_pink.value = centroid_x
                centr_y_pink.value = centroid_y
                centr_x.value = 0
                centr_y.value = 0
                # if centroid_y > 500:
                pink_b.value = True
                # red_b.value = False
                green_b.value = False

        elif pink_green:
            # if cv2.contourArea(max_cnt) > cv2.contourArea(max_cnt2):
            if (cv2.contourArea(max_cnt) > 1000 and cv2.contourArea(max_cnt) < 306000):
                # Draw a rectange on the contour
                rect = cv2.minAreaRect(max_cnt)
                box = cv2.boxPoints(rect)
                box = np.intp(box)
                cv2.drawContours(img, [box], -1, (255, 0, 0), 3)

                (x, y, w, h) = cv2.boundingRect(box)
                centroid_y = y + h // 2
                centroid_x = x + w // 2
                centr_y.value = centroid_y
                centr_x.value = centroid_x
                centr_x_red.value = 0
                centr_x_red.value = 0
                red_b.value = False
                green_b.value = True  # pink_b.value = False
            # elif cv2.contourArea(max_cnt2) > cv2.contourArea(max_cnt):
            if (cv2.contourArea(max_cnt2) > 2000 and cv2.contourArea(max_cnt2) < 306000):
                # Draw a rectange on the contour
                rect2 = cv2.minAreaRect(max_cnt2)
                box = cv2.boxPoints(rect2)
                box = np.intp(box)
                cv2.drawContours(img, [box], -1, (255, 0, 0), 3)
                (x, y, w, h) = cv2.boundingRect(box)
                centroid_x = x + w // 2
                centroid_y = y + h // 2
                centr_x_pink.value = centroid_x
                centr_y_pink.value = centroid_y
                centr_x_red.value = 0
                centr_y_red.value = 0
                # if centroid_y > 500:
                pink_b.value = True
                red_b.value = False  # green_b.value = False
        else:
            centr_x_pink.value = 0
            centr_y_red.value = 0
            centr_y.value = 0
            centr_x.value = 0
            centr_x_red.value = 0
        # print(f"Green:{green_present}, red:{red_present}, pink:{pink_present}")
        # print(f"all:{all_flag}, only_red:{only_red}, only_green:{only_green}, only_pink:{only_pink}, pink_green:{pink_green}, pink_red:{pink_red}, both:{both_flag}")
        # print(f"g_next:{g_next.value}, r_next:{r_next.value}")
        # print(f"green:{green_b.value}  red:{red_b.value}, pink:{pink_b.value}")
        # print(f"green centr :{centr_x.value}, red_centr:{centr_x_red.value}, pink_centr:{centr_x_pink.value}")
        # print(f"{centr_y_pink.value}")
        cv2.imshow('Object Dist Measure ', img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            stop_b.value = True
            break
    # print(dist2)

    cv2.destroyAllWindows()
    picam2.stop()


# pwm.set_PWM_dutycycle(12, power.value)


def servoDrive(pwm, color_b, stop_b, red_b, green_b, pink_b, counts, centr_y, centr_x, centr_y_red, centr_x_red,
               centr_x_pink, centr_y_pink, head):
    # print("ServoProcess started")
    global heading, imu, corr, corr_pos

    pb_time = 0
    pwm.set_mode(RX_Head, pigpio.INPUT)
    pwm.set_mode(RX_Left, pigpio.INPUT)
    pwm.set_mode(RX_Right, pigpio.INPUT)
    pwm.set_mode(12, pigpio.OUTPUT)  # Set pin 12 as an output
    pwm.set_mode(20, pigpio.OUTPUT)  # Set pin 20 as an output
    pwm.hardware_PWM(12, 100, 0)
    try:
        pwm.bb_serial_read_open(RX_Head, 115200)
        pwm.bb_serial_read_open(RX_Left, 115200)
        pwm.bb_serial_read_open(RX_Right, 115200)
    except pigpio.error as e:
        if e == 'GPIO already in use':
            pwm.stop()
            pwm.bb_serial_read_open(RX_Head, 115200)
            pwm.bb_serial_read_open(RX_Left, 115200)
            pwm.bb_serial_read_open(RX_Right, 115200)
    x = 0
    y = 0

    previous_state = 0
    button_state = 0
    button = False
    pwm.set_PWM_dutycycle(12, 0)  # Set duty cycle to 50% (128/255)

    heading_angle = 0

    trigger = False
    counter = 0
    correctAngle(heading_angle, head.value)
    enc = EncoderCounter()
    past_time = 0
    setPointL = -70
    setPointR = 70
    setPointC = 0
    reset_f = False
    change_path = False
    green_turn = False
    turn_t = 0
    current_time = 0
    timer_started = False
    power = 0
    prev_power = 0

    g_flag = False
    r_flag = False
    p_flag = False

    gp_time = 0
    rp_time = 0

    g_past = False
    r_past = False
    p_past = False

    red_stored = False
    green_stored = False

    prev_time = 0
    red_turn = False
    buff = 0

    blue_flag = False
    orange_flag = False

    color_n = ""
    parking_flag = False

    green_count = 0
    red_count = 0

    reverse = False

    parking_heading = False
    last_counter = 12
    c_time = 0
    stop_flag = False
    calc_time = False
    continue_parking = False

    lap_finish = False
    change_counter = 8
    pink_run = 1
    prev_distance = 0
    counter_reset = False
    turn_flag = False
    reset_flags = False
    offset = 0
    finished = False
    target_count = 0

    red_time = False
    green_time = False
    turn_trigger_distance = 0
    finish = False
    pink_detected = False
    centr_threshold = 800
    i = 0
    l = 0
    lap_finish_time = 0
    while True:
        ##### STOP CONDITION ######
        print(f"pink detected:{pink_detected}")
        if counter == last_counter and not lap_finish:
            # if ((green_count == 1 or red_count == 1) or (green_count == 0 and red_count == 0)) and not lap_finish:
            if not finished:
                target_count = counts.value + 30000
                finished = True
            if counts.value >= target_count:
                power = 0
                pwm.set_PWM_dutycycle(12, power)  # Set duty cycle to 50% (128/255)
                time.sleep(1)
                power = 100
                prev_power = 0
                lap_finish = True
                lap_finish_time = time.time()

        if lap_finish:
            if not counter_reset:
                counter = counter % 12
                counter_reset = True

        if lap_finish and not continue_parking:
            if orange_flag:
                setPointR = -100
                setPointC = -100
            elif blue_flag:
                setPointL = 100
                setPointC = 100

        if continue_parking:  ### THIS SETPOINT IS WHEN THE ROBOT IS IN THE PARKING MODE
            green_b.value = False
            red_b.value = False
            g_past = False
            r_past = False
            g_flag = False
            r_flag = False
            if orange_flag and (centr_x_pink.value < 1200 and centr_x_pink.value > 0):
                setPointR = -35
                setPointC = -35
                finish = True
            elif blue_flag and (centr_x_pink.value > 800 and centr_x_pink.value > 0):
                setPointL = 35
                setPointC = 35
                finish = True

        if pink_b.value:  ### DECIDES SETPOINT WHENEVER PINK IS IN THE FRAME
            print(f"Pink detected while green is there {pink_detected}")
            if orange_flag:
                if centr_x_pink.value < 1200 and centr_x_pink.value > 0 and not continue_parking:
                    if not pink_detected:
                        setPointL = setPointL + 35
                        pink_detected = True
                    # setPointL = -35
                    setPointR = 70
                    print(f"setPointL :{setPointL}")
            elif blue_flag:
                if centr_x_pink.value > 800 and centr_x_pink.value > 0 and not continue_parking:
                    if not pink_detected:
                        setPointR = setPointR - 35
                        pink_detected = True
                    setPointL = -70
            elif counter % 4 == 0 and not blue_flag and not orange_flag:
                if centr_x_pink.value > 800 and centr_x_pink.value > 0 and not continue_parking:
                    if not pink_detected:
                        setPointR = setPointR - 35
                        pink_detected = True
                    setPointL = -70
                if centr_x_pink.value < 800 and centr_x_pink.value > 0 and not continue_parking:
                    if not pink_detected:
                        setPointL = setPointL + 35
                        pink_detected = True
                    # setPointL = -35
                    setPointR = 70
            pb_time = time.time()

            if lap_finish and not continue_parking:
                print("Starting Parking...")
                continue_parking = True

        elif not pink_b.value and time.time() - pb_time > 1 and not lap_finish:  ### IF DOES NOT SEE PINK, KEEP THE SAME SETPOINT FOR 1 SECOND AND THEN CHANGE
            print(f"Resetting setPoints...{pink_detected}")
            pink_detected = False

        if (centr_x.value > 800 and centr_x.value > 0) and not lap_finish:
            print(f"away from green {g_past}")
            if not pink_b.value:
                setPointL = setPointL - 1
                setPointR = 70
        elif (centr_x_red.value < 1200 and centr_x_red.value > 0) and not lap_finish:
            print("away from red {r_past}")
            if not pink_b.value:
                setPointR = setPointR + 1
                setPointL = -70

        try:
            color_sensor = imu.get_color()

            if (imu.color_rgb[0] == 0 and imu.color_rgb[1] == 0 and imu.color_rgb[2] == 0):
                continue

            previous_state = button_state
            button_state = GPIO.input(5)

            if previous_state == 1 and button_state == 0:
                button = not (button)
                power = 100

            if button:  ##### THIS BLOCK OF CODE WHEN BUTTON IS PRESSED

                if not reverse:
                    imu_head = head.value
                else:
                    # print(f"Changing imu..{imu_head}")
                    imu_head = head.value - 180
                # print(f"after Changing imu..{imu_head}")
                # print(f"imu_heading in while:{imu_head}")
                if reset_flags:
                    if orange_flag:
                        blue_flag = True
                        orange_flag = False
                        color_n = "Blue"
                        reset_flags = False
                    elif blue_flag:
                        orange_flag = True
                        blue_flag = False
                        color_n = "Orange"
                        reset_flags = False

                x, y = enc.get_position(imu_head, counts.value)

                total_power = (power * 0.1) + (prev_power * 0.9)

                prev_power = total_power
                pwm.set_PWM_dutycycle(12, 2.55 * total_power)  # Set duty cycle to 50% (128/255)

                pwm.write(20, 1)  # Set pin 20 high

                if stop_b.value:
                    power = 0
                    prev_power = 0

                ######      DIRECTION DECISION (CLOCKWISE OR ANTICLOCKWISE)     #####

                if not blue_flag and not orange_flag:
                    if color_sensor == "Orange":
                        orange_flag = True
                        blue_flag = False
                        color_n = "Orange"

                    elif color_sensor == "Blue":
                        blue_flag = True
                        orange_flag = False
                        color_n = "Blue"

                ################        PARKING         ################

                if parking_flag and not stop_flag:
                    p_flag = False
                    p_past = False
                    print(f"PARKING ------> distance_head : {tfmini.distance_head}")
                    print("Inside Parking Loop")

                    if not calc_time:
                        c_time = time.time()
                        calc_time = True
                    while time.time() - c_time < 0.5:
                        print("Reversing backward...")
                        power = 100
                        prev_power = 0
                        pwm.set_PWM_dutycycle(12, power)  # Set duty cycle to 50% (128/255)
                        pwm.write(20, 0)  # Set pin 20 hig

                    power = 85
                    pwm.set_PWM_dutycycle(12, power)
                    if orange_flag:
                        if not parking_heading:
                            heading_angle = heading_angle - 90
                            parking_heading = True

                    elif blue_flag:
                        if not parking_heading:
                            heading_angle = heading_angle + 90
                            parking_heading = True

                    print(f"Correcting angle..{abs(corr)}")
                    correctAngle(heading_angle, head.value)
                    tfmini.getTFminiData()
                    if (abs(corr) < 5) or (tfmini.distance_head < 5 and tfmini.distance_head >= 0) and not stop_flag:
                        power = 0
                        prev_power = 0
                        servo.setAngle(90)
                        pwm.set_PWM_dutycycle(12, power)
                        stop_flag = True
                        print("Succesfully Parked...")

                else:
                    if reset_f:
                        setPointL = -70
                        setPointR = 70
                        if blue_flag:  ### BLUE RESET BLOCK
                            print("BLUE RESET...")

                            if red_b.value or red_turn:  # red after trigger
                                tfmini.getTFminiData()
                                x, y = enc.get_position(imu_head, counts.value)
                                print(
                                    f"Red Detected after trigger...green: {g_flag} {g_past} red:{r_flag} {r_past} {setPointR} {setPointL}")
                                red_turn = True

                                if pink_b.value and not red_b.value:
                                    red_turn = False
                                    red_time = False
                                elif tfmini.distance_head < 50 and not red_b.value and not pink_b.value:
                                    red_turn = False
                                    red_time = True
                                correctPosition(setPointC, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                reset_f, reverse, head.value, centr_x_pink.value, finish)

                            else:
                                if red_time:
                                    time_g = 1.2

                                else:
                                    time_g = 0.5

                                if not timer_started:
                                    current_time = time.time()
                                    timer_started = True

                                if not green_b.value and not g_past:
                                    print('reversing diection red')
                                    while (time.time() - current_time < time_g):
                                        servo.setAngle(120)
                                        x, y = enc.get_position(imu_head, counts.value)
                                        power = 100
                                        prev_power = 95
                                        pwm.set_PWM_dutycycle(12, power)  # Set duty cycle to 50% (128/255)
                                        pwm.write(20, 0)  # Set pin 20 hig
                                    print('reversing diection red complete')
                                    print('Stopping Motor...')
                                    tfmini.getTFminiData()
                                    turn_trigger_distance = tfmini.distance_head
                                    turn_cos_theta = math.cos(math.radians(abs(corr)))

                                elif (green_b.value or green_turn) or g_past:
                                    green_turn = True
                                    while 1 and green_b.value:
                                        correctAngle(heading_angle, head.value)
                                        if abs(corr) < 15:
                                            tfmini.getTFminiData()
                                            turn_trigger_distance = tfmini.distance_head
                                            turn_cos_theta = math.cos(math.radians(abs(corr)))
                                            break
                                    print('reversing diection Green')

                                    while 1:
                                        buff = 4
                                        if (green_b.value and (centr_y.value < 450 and centr_y.value > 0)):
                                            print(f"Breaking the loop...")
                                            break
                                        elif (time.time() - current_time > 1.5) and not green_b.value:
                                            print("Green is not there breaking the loop...")
                                            break
                                        x, y = enc.get_position(imu_head, counts.value)
                                        power = 100
                                        prev_power = 0
                                        pwm.set_PWM_dutycycle(12, power)  # Set duty cycle to 50% (128/255)
                                        pwm.write(20, 0)  # Set pin 20 hig
                                    print('Green reversing diection complete')
                                    print('Stopping Motor...')

                                power = 0
                                prev_power = 0
                                pwm.set_PWM_dutycycle(12, power)  # Set duty cycle to 50% (128/255)

                                tfmini.getTFminiData()
                                print(f"head: {tfmini.distance_head}")
                                print(f"before update: {x} {y}")
                                time.sleep(1)
                                counter = counter + 1
                                lane_reset = counter % 4
                                print(f"in Lane {lane_reset}")
                                if lane_reset == 1:
                                    enc.x = (150 - abs(turn_trigger_distance * turn_cos_theta)) - 10
                                if lane_reset == 2:
                                    enc.y = (abs(turn_trigger_distance * turn_cos_theta) - 250) + 10
                                if lane_reset == 3:
                                    enc.x = (abs(turn_trigger_distance * turn_cos_theta) - 150) + 10
                                if lane_reset == 0:
                                    enc.y = (50 - abs(turn_trigger_distance * turn_cos_theta)) - 10
                                print(f'Resuming Motor...{x} {y}')

                                power = 100
                                if reverse == True:
                                    print("In blue reverse...")
                                    offset = 180
                                    heading_angle = -((90 * counter) % 360) - offset
                                    if abs(heading_angle) >= 360:
                                        heading_angle = (heading_angle % 360)
                                else:
                                    print("In blue reverse else...")
                                    heading_angle = -((90 * counter) % 360)
                                green_turn = False
                                red_turn = False
                                reset_f = False
                                red_time = False
                                r_flag = False
                                r_past = False

                        if orange_flag:  ### ORANGE RESET BLOCK
                            print("ORANGE RESET...")

                            if green_b.value or green_turn:  # green after trigger
                                # if((green_b.value or green_turn) and centr_y.value < 700):
                                tfmini.getTFminiData()
                                x, y = enc.get_position(imu_head, counts.value)
                                print(f"Green Detected after trigger... ")
                                green_turn = True
                                if pink_b.value and not green_b.value:
                                    green_turn = False
                                    green_time = False
                                elif tfmini.distance_head < 50 and not green_b.value and not pink_b.value:
                                    green_turn = False
                                    green_time = True
                                correctPosition(setPointC, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                reset_f, reverse, head.value, centr_x_pink.value, finish)
                            else:
                                print("ORANGE RESET ELSE..")
                                if green_time:
                                    time_g = 1.2

                                else:
                                    time_g = 0.5
                                if not timer_started:
                                    current_time = time.time()
                                    timer_started = True

                                if not red_b.value and not r_past:
                                    print('reversing diection green')
                                    tfmini.getTFminiData()
                                    turn_trigger_distance = tfmini.distance_head
                                    while time.time() - current_time < time_g:
                                        servo.setAngle(70)
                                        # getTFminiData()
                                        x, y = enc.get_position(imu_head, counts.value)
                                        power = 100
                                        prev_power = 95
                                        pwm.set_PWM_dutycycle(12, power)  # Set duty cycle to 50% (128/255)
                                        pwm.write(20, 0)  # Set pin 20 hig
                                    print('reversing diection green complete')
                                    print('Stopping Motor...')
                                    tfmini.getTFminiData()
                                    turn_trigger_distance = tfmini.distance_head
                                    turn_cos_theta = math.cos(math.radians(abs(corr)))

                                elif (red_b.value or red_turn) or r_past:
                                    red_turn = True
                                    while 1:
                                        time.sleep(0.05)
                                        tfmini.getTFminiData()
                                        print(f"correct red heading..")
                                        correctAngle(heading_angle, head.value)
                                        if abs(corr) < 20:
                                            turn_trigger_distance = tfmini.distance_head
                                            print(f"turn_trigger: {turn_trigger_distance}")
                                            turn_cos_theta = math.cos(math.radians(corr))
                                            break

                                    print(
                                        f'reversing diection red pink color: {pink_b.value} pink flag: {p_flag} {p_past}  red color:{red_b.value} red flag: {r_past} {r_flag}')
                                    while 1:
                                        print("RED IS SEEN..")
                                        buff = 4
                                        servo.setAngle(70)
                                        print(f"centr y: {centr_y_red.value}")
                                        if (red_b.value and (centr_y_red.value < 450 and centr_y_red.value > 0)):
                                            print(f"Breaking the loop...")
                                            break
                                        elif (time.time() - current_time > 1.5) and not red_b.value:
                                            print("Green is not there breaking the loop...")
                                            break

                                        # getTFminiData()
                                        x, y = enc.get_position(imu_head, counts.value)
                                        # print(f"x: {x}, y: {y},  count:{counts.value} distance_head : {distance_head}")
                                        power = 100
                                        prev_power = 0
                                        pwm.set_PWM_dutycycle(12, power)  # Set duty cycle to 50% (128/255)
                                        pwm.write(20, 0)  # Set pin 20 hig
                                    print('red reversing diection complete')

                                print('Stopping Motor...')

                                power = 0
                                prev_power = 0
                                pwm.set_PWM_dutycycle(12, power)  # Set duty cycle to 50% (128/255)

                                # getTFminiData()
                                x, y = enc.get_position(imu_head, counts.value)

                                time.sleep(0.5)

                                counter = counter + 1
                                lane_reset = counter % 4
                                print(f"head: {turn_trigger_distance}, corr: {turn_cos_theta}")
                                if lane_reset == 1:
                                    enc.x = (150 - (turn_trigger_distance * turn_cos_theta)) - 10
                                    print(f"x: {enc.x}")
                                if lane_reset == 2:
                                    enc.y = (250 - (turn_trigger_distance * turn_cos_theta)) - 10
                                if lane_reset == 3:
                                    enc.x = ((turn_trigger_distance * turn_cos_theta) - 150) + 10
                                if lane_reset == 0:
                                    enc.y = ((turn_trigger_distance * turn_cos_theta) - 50) + 10
                                print(f'Resuming Motor...{offset}')

                                power = 100
                                if reverse == True:
                                    offset = -180
                                    heading_angle = ((90 * counter) % 360) + offset
                                    if abs(heading_angle) >= 360:
                                        heading_angle = (heading_angle % 360)
                                else:
                                    heading_angle = ((90 * counter) % 360)
                                red_turn = False
                                green_time = False
                                reset_f = False
                                g_flag = False
                                g_past = False
                        green_count = 0
                        red_count = 0
                    else:
                        if color_sensor == color_n and not trigger and (time.time() - turn_t) > (4 + buff):
                            buff = 0
                            timer_started = False
                            trigger = True
                            reset_f = True
                            turn_t = time.time()

                        elif color_sensor == 'White':
                            trigger = False

                        if counter == change_counter and green_count == 1 and not turn_flag:
                            change_path = False
                            power = 100
                            turn_flag = True
                        elif counter == change_counter and red_count == 1 and not turn_flag:
                            print("Changing path...")
                            reset_f = False
                            trigger = False
                            while time.time() - past_time < 1.5:
                                print("going 0")
                                correctPosition(setPointC, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                reset_f, reverse, head.value, centr_x_pink.value, finish)
                            change_path = True
                            power = 100
                            reverse = True
                            reset_flags = True
                            turn_flag = True

                        ###################### PANDAV #####################

                        if green_b.value and not g_flag and not r_flag:
                            g_flag = True
                            g_past = True
                            GPIO.output(10, GPIO.LOW)
                            GPIO.output(26, GPIO.LOW)
                            GPIO.output(6, GPIO.LOW)
                            print('1')

                        elif (g_past or time.time() - gp_time < 0.5):

                            if centr_x.value > 1200 and centr_x.value > 0:
                                tfmini.getTFminiData()
                            if tfmini.distance_right <= 50 and g_past:
                                g_past = False
                                g_flag = False
                                red_count = 0
                                green_count = 1
                                GPIO.output(26, GPIO.LOW)
                                GPIO.output(6, GPIO.HIGH)
                                GPIO.output(10, GPIO.LOW)
                                buff = 0
                                gp_time = time.time()
                            g_flag = True
                            print('2')

                        elif red_b.value and not r_flag and not g_flag:
                            r_flag = True
                            r_past = True
                            GPIO.output(26, GPIO.LOW)
                            GPIO.output(6, GPIO.LOW)
                            GPIO.output(10, GPIO.LOW)
                            print('3')

                        elif (r_past or time.time() - rp_time < 0.5):

                            if centr_x_red.value < 400 and centr_x_red.value > 0:
                                tfmini.getTFminiData()
                            if tfmini.distance_left <= 50 and r_past:
                                print(f"red Avoid complete")
                                r_past = False
                                r_flag = False
                                red_stored = False
                                red_count = 1
                                green_count = 0
                                GPIO.output(10, GPIO.HIGH)
                                GPIO.output(26, GPIO.LOW)
                                GPIO.output(6, GPIO.LOW)
                                buff = 0
                                rp_time = time.time()
                            r_flag = True
                            print('4')

                        elif pink_b.value and not p_past and continue_parking:
                            p_flag = True
                            p_past = True
                            print('5')

                        elif p_past and continue_parking and not parking_flag:
                            p_flag = True
                            tfmini.getTFminiData()
                            if orange_flag:
                                print(
                                    f"prev_distance: {prev_distance}, distance_left: {tfmini.distance_left} diff: {abs(prev_distance - tfmini.distance_left)}")

                                if tfmini.distance_left <= 30 and (
                                        abs(prev_distance - tfmini.distance_left) > 7 and prev_distance > 0) and p_past:
                                    p_past = False
                                    p_flag = False
                                    parking_flag = True
                                    print("Pink Avoidance Complete...")
                                prev_distance = tfmini.distance_left

                            elif blue_flag:
                                print(
                                    f"prev_distance: {prev_distance}, distance_right: {tfmini.distance_right} diff: {abs(prev_distance - tfmini.distance_right)}")

                                if tfmini.distance_right <= 30 and (
                                        abs(prev_distance - tfmini.distance_right) > 7 and prev_distance > 0) and p_past:
                                    p_past = False
                                    p_flag = False
                                    parking_flag = True
                                    print("Pink Avoidance Complete Blue...")
                                prev_distance = tfmini.distance_right
                            print('6')

                        else:
                            g_flag = False
                            r_flag = False
                            p_flag = False
                            r_past = False
                            g_past = False
                            p_past = False

                            print('7')

                            GPIO.output(10, GPIO.LOW)
                            GPIO.output(26, GPIO.LOW)
                            GPIO.output(6, GPIO.LOW)

                        if not change_path:

                            if g_flag:
                                correctPosition(setPointL, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                reset_f, reverse, head.value, centr_x_pink.value, finish)
                            elif r_flag:
                                print("avoiding red...")
                                correctPosition(setPointR, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                reset_f, reverse, head.value, centr_x_pink.value, finish)
                            elif p_flag:
                                if orange_flag:
                                    correctPosition(setPointR, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                    reset_f, reverse, head.value, centr_x_pink.value, finish)
                                elif blue_flag:
                                    correctPosition(setPointL, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                    reset_f, reverse, head.value, centr_x_pink.value, finish)
                            else:
                                correctPosition(setPointC, heading_angle, x, y, counter, blue_flag, orange_flag,
                                                reset_f, reverse, head.value, centr_x_pink.value, finish)




                        else:
                            print(f"Turning 180...")
                            if heading_angle == -180:
                                change_path = False
                                print("Change path is false")
                            offset = -90
                            correctAngle(heading_angle, head.value)
                            if abs(corr) < 15 and i < 2:
                                heading_angle = heading_angle + offset
                                i = i + 1

                # print(f"color_n:{color_n} color_sensor:{color_sensor} x: {x}, y: {y} counts: {counts.value}, prev_distance: {prev_distance}, distance_right: {distance_right}, distance_left: {distance_left}, imu: {imu_head}, heading: {heading_angle}, cp: {continue_parking}, counter: {counter}, pink_b: {pink_b.value} p_flag = {p_flag}, g_flag: {g_flag} r_flag: {r_flag} p_past: {p_past}, g_past: {g_past}, r_past: {r_past} , red_stored:{red_stored} green_stored:{green_stored}")
                print(
                    f"centr_x:{centr_x.value} centr_red: {centr_x_red.value} setPointL:{setPointL} setPointR:{setPointR} g_count:{green_count} r_count:{red_count} color_n:{color_n} color_sensor:{color_sensor} x: {x}, y: {y} counts: {counts.value}, prev_distance: {prev_distance}, distance_right: {tfmini.distance_right}, distance_left: {tfmini.distance_left}, imu: {imu_head}, heading: {heading_angle}, cp: {continue_parking}, counter: {counter}, pink_b: {pink_b.value} p_flag = {p_flag}, g_flag: {g_flag} r_flag: {r_flag} p_past: {p_past}, g_past: {g_past}, r_past: {r_past} , red_stored:{red_stored} green_stored:{green_stored}")

            # print(f"sp:{setPointL} {setPointR} x:{x}, y:{y}, heading:{heading_angle} {imu_head} color: {color_sensor}, orange:{orange_flag}, blue:{blue_flag}, corr:{corr}")  # print(f"x:{x} y:{y} offset:{offset} imu:{imu_head} heading_angle:{heading_angle} counter:{counter}, change_counter:{change_counter} g_count:{green_count}, r_count:{red_count}, change_path:{change_path}, turn_f:{turn_flag}")  # print(f"resewt:{reset_flags} color_n:{color_n} offset:{offset} imu: {imu_head}, counter:{counter} heading:{heading_angle} x:{x} y:{y}, green_count: {green_count}, red_count:{red_count}, change_path:{change_path}, last_counter:{last_counter}, blue:{blue_flag}, orange:{orange_flag}")  # print(f"reset_flags:{reset_flags} color_n:{color_n} trigger:{trigger} reset:{reset_f} offset:{offset} imu: {head.value}, counter:{counter} heading:{heading_angle} x:{x} y:{y}, green_count: {green_count}, red_count:{red_count}, change_path:{change_path}, last_counter:{last_counter}, blue:{blue_flag}, orange:{orange_flag}, centroid:{centr_x_pink.value}")  # print(f"red_count: {red_count} green_count:{green_count} counter:{counter} change_counter:{change_counter}, imu:{imu_head}, offset:{offset}, heading_angle:{heading_angle}")
            else:
                power = 0
                pwm.hardware_PWM(12, 100, 0)
                heading_angle = 0
                counter = 0
                correctAngle(heading_angle, head.value)
                color_b.Value = False
                stop_b.value = False
                red_b.value = False
                green_b.value = False

        except Exception as e:
            print(f"Exception: {e}")
            if isinstance(e, KeyboardInterrupt):
                power = 0
                pwm.hardware_PWM(12, 100, 0)
                heading_angle = 0
                counter = 0
                correctAngle(heading_angle, head.value)
                color_b.Value = False
                stop_b.value = False
                red_b.value = False
                green_b.value = False


def runEncoder(counts, head):
    print("Encoder Process Started")
    ser = serial.Serial('/dev/UART_USB', 115200)
    print("created uart")

    try:
        while True:
            # GPIO.output(26, HIGH)
            line = ser.readline().decode().strip()
            data = line.split(" ")
            try:
                # print(f"data:{data}")
                if data[0].isdigit() or data[1].isdigit():
                    # GPIO.output(10, LOW)
                    counts.value = int(data[1])
                    head.value = float(data[0])

                else:
                    pass
                # GPIO.output(10, HIGH)
            except ValueError:
                continue

    except KeyboardInterrupt:
        ser.close()


if __name__ == '__main__':
    try:
        pwm = pigpio.pi()
        counts = multiprocessing.Value('i', 0)
        color_b = multiprocessing.Value('b', False)
        stop_b = multiprocessing.Value('b', False)
        red_b = multiprocessing.Value('b', False)
        green_b = multiprocessing.Value('b', False)
        pink_b = multiprocessing.Value('b', False)
        centr_y = multiprocessing.Value('f', 0.0)
        centr_x = multiprocessing.Value('f', 0.0)
        centr_y_red = multiprocessing.Value('f', 0.0)
        centr_x_red = multiprocessing.Value('f', 0.0)
        centr_x_pink = multiprocessing.Value('f', 0.0)
        centr_y_pink = multiprocessing.Value('f', 0.0)
        head = multiprocessing.Value('f', 0.0)

        P = multiprocessing.Process(target=Live_Feed, args=(
        color_b, stop_b, red_b, green_b, pink_b, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink,
        centr_y_pink))
        S = multiprocessing.Process(target=servoDrive, args=(
        pwm, color_b, stop_b, red_b, green_b, pink_b, counts, centr_y, centr_x, centr_y_red, centr_x_red, centr_x_pink,
        centr_y_pink, head))
        E = multiprocessing.Process(target=runEncoder, args=(counts, head,))

        E.start()
        S.start()
        P.start()


    except KeyboardInterrupt:
        pwm.hardware_PWM(12, 100, 0)
        pwm.bb_serial_read_close(RX_Head)
        pwm.bb_serial_read_close(RX_Left)
        pwm.bb_serial_read_close(RX_Right)
        pwm.stop()
        imu.close()
        GPIO.cleanup()