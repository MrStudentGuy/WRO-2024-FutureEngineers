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


class IMUandColorSensor:
	def __init__(self, scl, sda, frequency=400000):
		self.i2c = busio.I2C(scl, sda, frequency=frequency)
		#self.i2c1 = busio.I2C(scl, sda)
		self.bno = BNO08X_I2C(self.i2c)
		#self.sensor = adafruit_tcs34725.TCS34725(self.i2c1)
		self.bno.enable_feature(BNO_REPORT_ACCELEROMETER)
		self.bno.enable_feature(BNO_REPORT_GYROSCOPE)
		self.bno.enable_feature(BNO_REPORT_MAGNETOMETER)
		self.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
		#self.sensor.gain = 60
		self.color_rgb = [0, 0, 0]

	def find_heading(self, dqw, dqx, dqy, dqz):
		norm = sqrt(dqw * dqw + dqx * dqx + dqy * dqy + dqz * dqz)
		try:
			dqw = dqw / norm
			dqx = dqx / norm
			dqy = dqy / norm
			dqz = dqz / norm
		except ZeroDivisionError:
			print("Zero Division Erroorr")
			pass
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

	def read_imu(self):
		quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
		heading = self.find_heading(quat_real, quat_i, quat_j, quat_k)
		return heading

	def get_color(self):
		#time.sleep(0.001)
		self.color_rgb = self.sensor.color_rgb_bytes
		if (self.color_rgb[2] < 20 and self.color_rgb[0] < 30 and self.color_rgb[1] < 20):
			return "Blue"
		elif (self.color_rgb[2] < 10 and self.color_rgb[0] > 80 and self.color_rgb[1] < 20):
			return "Orange"
		else:
			return "White"

	def close(self):
		GPIO.cleanup()