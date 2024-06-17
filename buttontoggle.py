import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17, GPIO.OUT)
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def ledtoggle():
    if(input == 1 and led != input):
		print("Toggle pin HIGH, ", end='')
		led = input
		led_last = not led_last
		GPIO.output(17, int(led_last))
		print("LED turned on? : " + str(led_last))
	elif(input == 0 and led != input):
		print("Toggle pin LOW")
		led = 0


if __name__ == '__main__':

    led = 0
    led_last = False

    while True:
        input = GPIO.input(18)
    GPIO.cleanup()
