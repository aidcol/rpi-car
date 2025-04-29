"""
This file makes use of code from the following sources:

User raja_961, “Autonomous Lane-Keeping Car Using Raspberry
Pi and OpenCV”. Instructables. URL:
https://www.instructables.com/Autonomous-Lane-Keeping-Car-U
sing-Raspberry-Pi-and/

Big Brains - ELEC 424 Final Project - Team 11. URL:
https://www.hackster.io/big-brains/big-brains-elec-424-final-project-team-11-53c862

"""
import time
import math

import RPi.GPIO as GPIO
import numpy as np
import cv2


GPIO.setmode(GPIO.BCM)

# ESC parameters
speedPin = 18
speed_pwm_hz = 50
speed_dc = 7.5
GPIO.setup(speedPin, GPIO.OUT)
speed = GPIO.PWM(speedPin, speed_pwm_hz)
speed.start(speed_dc)

# Steering parameters
steeringPin = 19
steering_pwm_hz = 50
steering_dc = 7.5
steering_right_dc = 6
steering_left_dc = 9
GPIO.setup(steeringPin, GPIO.OUT)
steering = GPIO.PWM(steeringPin, steering_pwm_hz)
steering.start(steering_dc)

# Video parameters
ESC_KEY_CODE = 27
VIDEO_DEVICE_IDX = 0
VIDEO_WIDTH = 160
VIDEO_HEIGHT = 120

# Optical encoder parameters
TIMESTAMP_PATH = "/sys/devices/platform/slay_device/speed"
prev_timestamp = -1

# Steering control ############################################################
def set_steering(p, steering):
    p.ChangeDutyCycle(steering)


def reset_steering(p):
    p.ChangeDutyCycle(7.5)


def calibrate_steering(p):
    print('Calibrating steering...')
    p.ChangeDutyCycle(6)
    time.sleep(2)
    p.ChangeDutyCycle(9)
    time.sleep(2)
    reset_steering(p)
    print('Steering calibration complete')

###############################################################################

# ESC speed control ###########################################################
def set_esc(p, speed):
    p.ChangeDutyCycle(speed)


def reset_esc(p):
    p.ChangeDutyCycle(7.5)


def calibrate_esc(p):
    print('Calibrating ESC...')
    p.ChangeDutyCycle(10)
    time.sleep(2)
    p.ChangeDutyCycle(5)
    time.sleep(2)
    reset_esc(p)
    print('ESC calibration complete')

###############################################################################

# Image processing functions ##################################################
def detect_edges(frame):
    # blue color detection
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # cv2.imshow("HSV",hsv)
    lower_blue = np.array([90, 120, 0], dtype = "uint8") # lower limit of blue color
    upper_blue = np.array([150, 255, 255], dtype="uint8") # upper limit of blue color
    mask = cv2.inRange(hsv,lower_blue,upper_blue) # this mask will filter out everything but blue

    # detect edges
    edges = cv2.Canny(mask, 50, 100) 
    # cv2.imshow("edges",edges)
    return edges

def region_of_interest(edges):
    height, width = edges.shape # extract the height and width of the edges frame
    mask = np.zeros_like(edges) # make an empty matrix with same dimensions of the edges frame

    # only focus lower half of the screen
    # specify the coordinates of 4 points (lower left, upper left, upper right, lower right)
    polygon = np.array([[
        (0, height), 
        (0,  height/2),
        (width , height/2),
        (width , height),
    ]], np.int32)

    cv2.fillPoly(mask, polygon, 255) # fill the polygon with blue color 
    cropped_edges = cv2.bitwise_and(edges, mask) 
    cv2.imshow("roi",cropped_edges)
    return cropped_edges

def detect_line_segments(cropped_edges):
    rho = 1  
    theta = np.pi / 180  
    min_threshold = 10 
    line_segments = cv2.HoughLinesP(cropped_edges, rho, theta, min_threshold, 
                                    np.array([]), minLineLength=5, maxLineGap=0)
    return line_segments

def average_slope_intercept(frame, line_segments):
    lane_lines = []

    if line_segments is None:
        print("no line segment detected")
        return lane_lines

    height, width,_ = frame.shape
    left_fit = []
    right_fit = []
    boundary = 1/3

    left_region_boundary = width * (1 - boundary) 
    right_region_boundary = width * boundary 

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                print("skipping vertical lines (slope = infinity)")
                continue

            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = (y2 - y1) / (x2 - x1)
            intercept = y1 - (slope * x1)

            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    # lane_lines is a 2-D array consisting the coordinates of the right and left lane lines
    # for example: lane_lines = [[x1,y1,x2,y2],[x1,y1,x2,y2]]
    # where the left array is for left lane and the right array is for right lane 
    # all coordinate points are in pixels
    return lane_lines

def make_points(frame, line):
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 / 2)  # make points from middle of the frame down

    if slope == 0: 
        slope = 0.1    

    x1 = int((y1 - intercept) / slope)
    x2 = int((y2 - intercept) / slope)

    return [[x1, y1, x2, y2]]

def display_lines(frame, lines, line_color=(0, 255, 0), line_width=6): # line color (B,G,R)
    line_image = np.zeros_like(frame)

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_image, (x1, y1), (x2, y2), line_color, line_width)

    line_image = cv2.addWeighted(frame, 0.8, line_image, 1, 1)  
    return line_image

def get_steering_angle(frame, lane_lines):
     height, width, _ = frame.shape

     if len(lane_lines) == 2: # if two lane lines are detected
        _, _, left_x2, _ = lane_lines[0][0] # extract left x2 from lane_lines array
        _, _, right_x2, _ = lane_lines[1][0] # extract right x2 from lane_lines array
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)  

     elif len(lane_lines) == 1: # if only one line is detected
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)

     elif len(lane_lines) == 0: # if no line is detected
        x_offset = 0
        y_offset = int(height / 2)

     angle_to_mid_radian = math.atan(x_offset / y_offset)
     angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  
     steering_angle = angle_to_mid_deg + 90 

     return steering_angle

def display_heading_line(frame, steering_angle, line_color=(0, 0, 255), line_width=5):
    heading_image = np.zeros_like(frame)
    height, width, _ = frame.shape

    steering_angle_radian = steering_angle / 180.0 * math.pi
    x1 = int(width / 2)
    y1 = height
    x2 = int(x1 - height / 2 / math.tan(steering_angle_radian))
    y2 = int(height / 2)

    cv2.line(heading_image, (x1, y1), (x2, y2), line_color, line_width)

    heading_image = cv2.addWeighted(frame, 0.8, heading_image, 1, 1)

    return heading_image

###############################################################################

# Set up GPIO
calibrate_esc(speed)
calibrate_steering(steering)

# Set up video stream
print("Initializing video stream...")
video = cv2.VideoCapture(VIDEO_DEVICE_IDX)
video.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
print("Video stream initialized.")
time.sleep(1)

# The loop
while True:
    ret, frame = video.read()
    cv2.imshow("original", frame)

    # Calling the functions
    edges = detect_edges(frame)
    roi = region_of_interest(edges)
    line_segments = detect_line_segments(roi)
    lane_lines = average_slope_intercept(frame,line_segments)

    lane_lines_image = display_lines(frame,lane_lines)
    cv2.imshow("lane_lines",lane_lines_image)

    steering_angle = get_steering_angle(frame, lane_lines)
    heading_image = display_heading_line(lane_lines_image,steering_angle)

    # Read the timestamp from the optical encoder and calculate the time difference
    time_diff_ns = 0
    with open(TIMESTAMP_PATH, "r") as f:
      curr_timestamp = int(f.read())
      if prev_timestamp == -1:
        prev_timestamp = curr_timestamp
      else:
        time_diff_ns = curr_timestamp - prev_timestamp
        prev_timestamp = curr_timestamp
        # print("time_diff: ", time_diff_ns)

    key = cv2.waitKey(1)
    if key == ESC_KEY_CODE:
      break

video.release()
cv2.destroyAllWindows()
