import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(17, GPIO.OUT)  # LED
GPIO.setup(18, GPIO.IN, pull_up_down=GPIO.PUD_UP)  # Button

# Set up motor driver pins
DIR_PIN = 27
PWM_PIN = 22
GPIO.setup(DIR_PIN, GPIO.OUT)
GPIO.setup(PWM_PIN, GPIO.OUT)

# Set initial state
led_state = GPIO.LOW
motor_state = GPIO.LOW
GPIO.output(17, led_state)
GPIO.output(DIR_PIN, motor_state)

# Motor control function
pwm = GPIO.PWM(PWM_PIN, 100)  # Initialize PWM with 100Hz frequency
pwm.start(0)  # Start with 0% duty cycle

def toggle_motor():
    global motor_state
    motor_state = not motor_state
    GPIO.output(DIR_PIN, motor_state)
    if motor_state:
        pwm.ChangeDutyCycle(50)  # Set duty cycle to 50% to run the motor
    else:
        pwm.ChangeDutyCycle(0)  # Set duty cycle to 0% to stop the motor

def toggle_led():
    global led_state
    led_state = not led_state
    GPIO.output(17, led_state)
    toggle_motor()  # Toggle motor when LED state changes

if __name__ == '__main__':
    print("Press the button to toggle the LED and motor")
    print("Ctrl+C to exit")

    try:
        while True:
            buttonstate = GPIO.input(18)
            if buttonstate == 0:
                toggle_led()
                time.sleep(0.175)  # Debounce delay
    except KeyboardInterrupt:
        pass
    finally:
        pwm.stop()
        GPIO.cleanup()