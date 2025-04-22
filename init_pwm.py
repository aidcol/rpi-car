import time

import RPi.GPIO as GPIO


def setup_gpio():
    GPIO.setmode(GPIO.BCM)

    # Speed control
    speedPin = 18
    speed_pwm_hz = 50
    speed_duty_cycle = 50
    GPIO.setup(speedPin, GPIO.OUT)
    speed = GPIO.PWM(speedPin, speed_pwm_hz)
    speed.start(speed_duty_cycle)

    # Steering control
    steeringPin = 19
    steering_pwm_hz = 50
    steering_duty_cycle = 7.5
    GPIO.setup(steeringPin, GPIO.OUT)
    steering = GPIO.PWM(steeringPin, steering_pwm_hz)
    steering.start(steering_duty_cycle)


if __name__ == '__main__':
    print('Initializing PWM...')
    setup_gpio()

    while True:
        time.sleep(1)
        print('hey')
