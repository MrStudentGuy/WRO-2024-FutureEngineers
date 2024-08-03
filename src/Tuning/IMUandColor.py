import time
import board
import busio
from math import atan2, sqrt, pi
import adafruit_tcs34725

from adafruit_bno08x import (
	BNO_REPORT_ACCELEROMETER,
	BNO_REPORT_GYROSCOPE,
	BNO_REPORT_MAGNETOMETER,
	BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
i2c1 = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_tcs34725.TCS34725(i2c)
bno = BNO08X_I2C(i2c1)
bno.enable_feature(BNO_REPORT_ACCELEROMETER)
bno.enable_feature(BNO_REPORT_GYROSCOPE)
bno.enable_feature(BNO_REPORT_MAGNETOMETER)
bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
sensor.gain = 60
prev_time = 0


def find_heading(dqw, dqx, dqy, dqz):
	norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz)
	dqw = dqw / norm
	dqx = dqx / norm
	dqy = dqy / norm
	dqz = dqz / norm

	ysqr = dqy * dqy

	t3 = +2.0 * (dqw * dqz + dqx * dqy)
	t4 = +1.0 - 2.0 * (ysqr + dqz * dqz)
	yaw_raw = atan2(t3, t4)

	yaw = yaw_raw * 180.0 / pi
	yaw = yaw - 180

	if yaw > 0:
		yaw = 360 - yaw
	else:
		yaw = abs(yaw)
	return yaw  # heading in 360 clockwise


while True:
	#print(1 / (time.time() - prev_time))
	#prev_time = time.time()
	try:
		color_rgb = sensor.color_rgb_bytes
		#time.sleep(0.1)
		# quat_i, quat_j, quat_k, quat_real = bno.quaternion
		# print("Rotation Vector Quaternion:")
		# heading = find_heading(quat_real, quat_i, quat_j, quat_k)
		if (color_rgb[0] == 0 and color_rgb[1] == 0 and color_rgb[2] == 0):
			continue
		elif color_rgb[2] < 20 and color_rgb[0] < 30 and color_rgb[1] < 20:
			color_n = "Blue"
		elif color_rgb[2] < 10 and color_rgb[0] > 80 and color_rgb[1] < 20:
			color_n = "Orange"
		else:
			color_n = "White"
		print("Color: {}".format(color_n))
		print(f"r: {color_rgb[0]} g:{color_rgb[1]} b:{color_rgb[2]}")
	except Exception as e:
		pass

GPIO.cleanup()