import RPi.GPIO as GPIO
import pigpio
import time

# Initialising pins
SERVO = 17

# Motor control function
pwm = pigpio.pi()
pwm.set_mode(SERVO, pigpio.OUTPUT)
pwm.set_PWM_frequency(SERVO, 50)

print("0 deg")
pwm.set_servo_pulsewidth(SERVO, 500)
time.sleep(3)

print("90 deg")
pwm.set_servo_pulsewidth(SERVO, 1500)
time.sleep(3)

print("180 deg")
pwm.set_servo_pulsewidth(SERVO, 2500)
time.sleep(3)

# turning off servo
pwm.set_PWM_dutycycle(SERVO, 0)
pwm.set_PWM_frequency(SERVO, 0)