import serial
import math

# Open the serial port
ser = serial.Serial('/dev/ttyUSB0', 115200)
# data = input()
# rad = str(math.radians(float(data)))
while True:
    # Rea	d data from ESP32

    # ser.write(rad.encode())
    esp_data = int(ser.readline().decode().strip())
    # esp_data = esp_data + 1
    # if esp_data.startswith("X: "):
    # x, y = esp_data.split(" ")
    # x = float(x.split("")[1])
    # y = float(y)

    # print(f"Received X: {x}, Y: {y}")
    print(f" ESP: {esp_data}, type:{type(esp_data)}")