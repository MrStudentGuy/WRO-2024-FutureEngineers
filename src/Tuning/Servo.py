from Servo import Servo


ser = Servo(8)

if __name__ == "__main__":
	try:
		angle = int(input("Angle = "))
		ser.setAngle(angle)
	except Exception as e:
		print(f"Exception: {e}")