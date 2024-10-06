import os

# Kill any existing instance of pigpiod and start it again to control GPIO pins
os.system("sudo pkill pigpiod")
os.system("sudo pigpiod")

import RPi.GPIO as GPIO
import time
import multiprocessing
import pigpio
import board
from imports.BNO085 import IMUandColorSensor  # Importing the IMU and Color sensor classes
from imports.Encoder import EncoderCounter   # Importing the encoder counter

import serial
import sys

# Redirect standard output to a log file
log_file = open('/home/pi/WRO-2024-FutureEngineers/Logs/log_open.txt', 'w')
sys.stdout = log_file

# Set GPIO mode to BCM
GPIO.setmode(GPIO.BCM)
GPIO.setup(14, GPIO.OUT)

# Reset device by toggling GPIO pin 14
print("Resetting....")
GPIO.output(14, GPIO.LOW)
time.sleep(1)
GPIO.output(14, GPIO.HIGH)

time.sleep(1)
print("Reset Complete")

# Initialize global variables for various tasks
glob = 0
ser = serial.Serial('/dev/UART_USB', 115200)  # Initialize serial communication with UART_USB at 115200 baud rate

# Setup GPIO pin 5 for button input with pull-up resistor
GPIO.setup(5, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Parameters for servo control
servo = 8
RX_Head = 23  # RX pins for receiving data from sensors
RX_Left = 24
RX_Right = 25

# Variables for servo control and PID calculation
dist = 15
focal = 1120
pixels = 30
width = 4
dist1 = 0
dist2 = 0
currentAngle = 0
error_gyro = 0
prevErrorGyro = 0
totalErrorGyro = 0
correction = 0
totalError = 0
prevError = 0

# PID control constants
kp = 0.6
ki = 0.1
kd = 0.5
setPoint_flag = 0

# Distance variables for the sensors
distance_head = 0
distance_left = 0
distance_right = 0

# Function to retrieve data from TFMini Plus LiDAR sensors
def getTFminiData():
    time.sleep(0.01)  # Wait briefly to prevent overwhelming the sensor
    # Reading data from the three sensors connected to RX_Head, RX_Left, RX_Right
    (count_head, recv_head) = pwm.bb_serial_read(RX_Head)
    (count_left, recv_left) = pwm.bb_serial_read(RX_Left)
    (count_right, recv_right) = pwm.bb_serial_read(RX_Right)

    # Process data for head sensor
    if count_head > 8:
        for i in range(0, count_head - 9):
            if recv_head[i] == 89 and recv_head[i + 1] == 89:  # 0x59 is the header of valid data
                checksum = sum(recv_head[i:i+8]) % 256
                if checksum == recv_head[i + 8]:
                    global distance_head
                    distance_head = recv_head[i + 2] + recv_head[i + 3] * 256  # Extract distance data

    # Process data for left sensor
    if count_left > 8:
        for i in range(0, count_left - 9):
            if recv_left[i] == 89 and recv_left[i + 1] == 89:
                checksum = sum(recv_left[i:i+8]) % 256
                if checksum == recv_left[i + 8]:
                    global distance_left
                    distance_left = recv_left[i + 2] + recv_left[i + 3] * 256  # Extract distance data

    # Process data for right sensor
    if count_right > 8:
        for i in range(0, count_right - 9):
            if recv_right[i] == 89 and recv_right[i + 1] == 89:
                checksum = sum(recv_right[i:i+8]) % 256
                if checksum == recv_right[i + 8]:
                    global distance_right
                    distance_right = recv_right[i + 2] + recv_right[i + 3] * 256  # Extract distance data

# Function to control and correct the angle based on sensor data
def correctAngle(setPoint_gyro, left, right, trigger, heading):
    time.sleep(0.01)  # Small delay for stability
    global glob, distance_right, distance_head, distance_left, corr, prev_time
    error_gyro = heading - setPoint_gyro  # Calculate the gyro error

    if error_gyro > 180:
        error_gyro = error_gyro - 360  # Adjust error if it exceeds 180 degrees

    # PID controller calculation
    pTerm = kp * error_gyro
    dTerm = kd * (error_gyro - prevErrorGyro)
    totalErrorGyro += error_gyro
    iTerm = ki * totalErrorGyro
    correction = pTerm + iTerm + dTerm

    getTFminiData()  # Get sensor data to adjust the correction
    if distance_left < 15:
        print("Inside Left")
        correction = correction - 20  # Modify correction if the left sensor detects an obstacle

    elif distance_right < 15:
        print("Inside Right")
        correction = correction + 20  # Modify correction if the right sensor detects an obstacle

    correction = max(min(correction, 30), -30)  # Limit correction range
    print(f"time : {time.time() - prev_time} imu: {glob} correction : {correction} error: {error_gyro} left: {distance_left}, right:{distance_right}")
    prev_time = time.time()
    prevErrorGyro = error_gyro
    setAngle(90 - correction)  # Set the servo to the calculated angle

# Function to set the servo angle using PWM
def setAngle(angle):
    pwm.set_servo_pulsewidth(servo, 500 + round(angle * 11.11))  # Convert angle to pulse width for the servo

# Function to manage servo movement based on distance and encoder data
def servoDrive(distance, block, pwm, counts, head):
    print("ServoProcess started")
    global heading, distance_right, distance_head, distance_left

    pwm.set_mode(servo, pigpio.OUTPUT)
    pwm.set_PWM_frequency(servo, 50)

    # Setup for serial communication with the LiDAR sensors
    pwm.set_mode(RX_Head, pigpio.INPUT)
    pwm.set_mode(RX_Left, pigpio.INPUT)
    pwm.set_mode(RX_Right, pigpio.INPUT)
    pwm.bb_serial_read_open(RX_Head, 115200)
    pwm.bb_serial_read_open(RX_Left, 115200)
    pwm.bb_serial_read_open(RX_Right, 115200)

    # Initialize motor PWM outputs
    pwm.set_mode(12, pigpio.OUTPUT)
    pwm.set_mode(20, pigpio.OUTPUT)
    pwm.hardware_PWM(12, 100, 0)
    pwm.set_PWM_dutycycle(12, 0)  # Initially set motor power to 0

    previous_state = 0
    button_state = 0
    button = False
    power = 70
    prev_power = 0
    counter = 0
    turn_flag = False
    heading_angle = 0
    target_angle = 0
    trigger = False
    left_flag = False
    right_flag = False
    correctAngle(0, left_flag, right_flag, trigger, head.value)

    try:
        while True:
            init_flag = False
            getTFminiData()  # Update sensor data
            previous_state = button_state
            button_state = GPIO.input(5)  # Check button state

            if previous_state == 1 and button_state == 0:  # Button press detected
                button = not button  # Toggle button state
                init_flag = True
                power = 100

                print("Button is pressed")

            if button:
                total_power = (power * 0.1) + (prev_power * 0.9)
                prev_power = total_power
                pwm.set_PWM_dutycycle(12, 2.55 * total_power)  # Adjust motor power

                pwm.write(20, 1)  # Set motor direction to forward

                # Check if the robot should turn based on sensor readings
                if not right_flag and not left_flag:
                    if distance_right > 100:
                        right_flag = True
                    elif distance_left > 100:
                        left_flag = True

                correctAngle(heading_angle, left_flag, right_flag, trigger, head.value)

                # Check for stopping conditions
                if counter == 12:
                    if distance_head < 150 and (distance_left + distance_right) < 100:
                        power = 0
                        pwm.set_PWM_dutycycle(12, power)
                        sys.exit()  # Exit process when condition met

                # Handle turns based on sensor data
                if right_flag:
                    if (distance_right > 100 and distance_head < 75) and not trigger:
                        counter += 1
                        heading_angle = (90 * counter) % 360
                        trigger = True
                    if distance_right < 85 and distance_head > 75:
                        trigger = False
               
		elif left_flag:
			if (distance_left > 100 and distance_head < 75) and not trigger:
				# time.sleep(0.5)
				counter = counter + 1
				heading_angle = -((90 * counter) % 360)
				trigger = True

			elif distance_left < 85 and distance_head > 75:
				trigger = False

			else:
				if init_flag:
					init_flag = False
				power = 0
				pwm.set_PWM_dutycycle(12, 0)
				pwm.hardware_PWM(12, 100, 0)
				heading_angle = 0
				counter = 0
				left_flag = False
				right_flag = False
				# counts.value = 0
				correctAngle(heading_angle, left_flag, right_flag, trigger, head.value)
			print(f"heading:{head.value} {heading_angle}  counter:{counter} {trigger},  target_count:{target_count}, encoder_c:{counts.value}, L C R:{distance_left} {distance_head} {distance_right}")
	except KeyboardInterrupt:
		imu = IMUandColorSensor(board.SCL, board.SDA)
		power = 0
		pwm.set_PWM_dutycycle(12, 0)
	except Exception as e:
		print(f"Exception: {e}")
		if e == 'OSError' or e == 'Input/output error':
			# time.sleep(0.001)
			imu = IMUandColorSensor(board.SCL, board.SDA)


def runEncoder(counts, head):
    # This function runs in a separate process to read encoder values from the serial port.
    # The encoder sends data via UART which includes the current heading and the encoder count.
    print("Encoder Process Started")

    try:
        while True:
            line = ser.readline().decode().strip()  # Read data from UART, decode, and strip any extra spaces
            data = line.split(" ")  # Split the data into separate values based on spaces
            try:
                if data[0].isdigit() or data[1].isdigit():  # Check if the data received is valid
                    counts.value = int(data[1])  # Store encoder count
                    head.value = float(data[0])  # Store heading from the IMU
                else:
                    pass  # If the data is not valid, continue to the next loop iteration
            except ValueError:
                continue  # Handle cases where decoding or data formatting fails

    except KeyboardInterrupt:
        ser.close()  # Close the serial connection when the process is interrupted

if __name__ == "__main__":
    # This block initializes multiprocessing for controlling the servo and reading encoder data.
    try:
        pwm = pigpio.pi()  # Initialize the pigpio library
        distance = multiprocessing.Value("f", 0.0)  # Shared float variable for distance
        block = multiprocessing.Value("i", 0)  # Shared integer variable for block
        counts = multiprocessing.Value('i', 0)  # Shared integer variable for encoder counts
        head = multiprocessing.Value('f', 0.0)  # Shared float variable for heading angle

        # Create two separate processes for servo control and encoder reading
        S = multiprocessing.Process(target=servoDrive, args=(distance, block, pwm, counts, head,))
        E = multiprocessing.Process(target=runEncoder, args=(counts, head,))
        S.start()  # Start the servo control process
        E.start()  # Start the encoder reading process

    except KeyboardInterrupt:
        # Handle any interruptions and clean up
        power = 0
        pwm.set_PWM_dutycycle(12, 0)  # Stop the PWM signal for motor control
        pwm.bb_serial_read_close(RX_Head)  # Close serial communication for the head sensor
        pwm.bb_serial_read_close(RX_Left)  # Close serial communication for the left sensor
        pwm.bb_serial_read_close(RX_Right)  # Close serial communication for the right sensor
        pwm.stop()  # Stop the pigpio library
        imu = IMUandColorSensor(board.SCL, board.SDA)  # Reinitialize the IMU and Color Sensor
        imu.close()  # Close the IMU sensor connection
        pwm.ChangeDutyCycle(0)  # Ensure no further PWM signals are sent
        GPIO.cleanup()  # Clean up GPIO pins to avoid conflicts for future executions
