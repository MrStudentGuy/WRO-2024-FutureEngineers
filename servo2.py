import pigpio
import time

# Initialising servo
SERVO = 17

pwm = pigpio.pi()
pwm.set_mode(SERVO, pigpio.OUTPUT)
pwm.set_PWM_frequency(SERVO, 50)

# Calculating scale factor to convert degree inputs to PWM range
scale_factor = (2500-500)/(180-0)

# Ensures it is run only as a script, not an import
if __name__ == '__main__':
    print("Enter angle in degrees (0-180)")
    print("Ctrl+C to exit")

    try:
        while True:
            # Takes angle input
            angle = int(input())

            # Checks for invalid input
            if angle < 0 or angle > 180:
                print('Invalid input')
                break

            # Uses scale factor to set servo PWM to input angle
            pwm.set_servo_pulsewidth(SERVO, (angle*scale_factor))

    except KeyboardInterrupt:
        # Turning off servo
        pwm.set_PWM_dutycycle(SERVO, 0)
        pwm.set_PWM_frequency(SERVO, 0)