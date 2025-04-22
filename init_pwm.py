import time

import RPi.GPIO as GPIO


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
GPIO.setup(steeringPin, GPIO.OUT)
steering = GPIO.PWM(steeringPin, steering_pwm_hz)
steering.start(steering_dc)

try:
    while True:
        time.sleep(10)
except KeyboardInterrupt:
    speed.stop()
    steering.stop()
    GPIO.cleanup()

