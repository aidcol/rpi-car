"""
This file has code taken from and influenced by the following sources:
User raja_961, “Autonomous Lane-Keeping Car Using Raspberry
Pi and OpenCV”. Instructables. URL:
https://www.instructables.com/Autonomous-Lane-Keeping-Car-U
sing-Raspberry-Pi-and/

"""


import time

import RPi.GPIO as GPIO


def reset_esc(p):
    p.ChangeDutyCycle(7.5)


GPIO.setmode(GPIO.BCM)

# Speed control
speedPin = 18
speed_pwm_hz = 50
speed_dc = 7.5
GPIO.setup(speedPin, GPIO.OUT)
speed = GPIO.PWM(speedPin, speed_pwm_hz)
speed.start(speed_dc)

# Steering control
steeringPin = 19
steering_pwm_hz = 50
steering_dc = 7.5
steering_right_dc = 6
steering_left_dc = 9
GPIO.setup(steeringPin, GPIO.OUT)
steering = GPIO.PWM(steeringPin, steering_pwm_hz)
steering.start(steering_dc)

# ESC calibration
print('Calibrating ESC...')

speed_dc = 10
speed.ChangeDutyCycle(speed_dc)
time.sleep(2)

speed_dc = 5
speed.ChangeDutyCycle(speed_dc)
time.sleep(2)

reset_esc(speed)

print('ESC calibration complete')

print('Running ESC demo...')
print('Spinning foward')
speed_dc = 8
speed.ChangeDutyCycle(speed_dc)
time.sleep(5)

print('Stopping')
reset_esc(speed)
time.sleep(2)

print('Running servo demo...')
print('Turning right')
steering.ChangeDutyCycle(steering_right_dc)
time.sleep(5)

print('Turning left')
steering.ChangeDutyCycle(steering_left_dc)
time.sleep(5)

print('Resetting')
steering.ChangeDutyCycle(steering_dc)

print('Running...')
try:
    while True:
        time.sleep(10)
except KeyboardInterrupt:
    speed.stop()
    steering.stop()
    GPIO.cleanup()

