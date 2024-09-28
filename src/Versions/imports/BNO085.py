import time
import board
import busio
from math import atan2, sqrt, pi
import adafruit_tcs34725
import math

class IMUandColorSensor:
	def __init__(self, scl, sda, frequency=400000):
		self.i2c1 = busio.I2C(scl, sda)
		self.prev_imu = 0

		self.sensor = adafruit_tcs34725.TCS34725(self.i2c1)
		self.sensor.gain = 60
		self.color_rgb = [0, 0, 0]


	def get_color(self):
		time.sleep(0.01)
		self.color_rgb = self.sensor.color_rgb_bytes
		if self.color_rgb[0] <= 35 and self.color_rgb[2] > 12:
			return "Blue"
		elif (self.color_rgb[0] >= 50 and self.color_rgb[0] <= 80) and self.color_rgb[2] < 15:
			return  "Orange"
		else:
			return "White"

	def close(self):
		GPIO.cleanup()