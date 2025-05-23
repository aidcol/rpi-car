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

import matplotlib.pyplot as plt  # pip install matplotlib


GPIO.setmode(GPIO.BCM)

# ESC parameters
speedPin = 18
speed_pwm_hz = 50
speed_dc = 7.5
GPIO.setup(speedPin, GPIO.OUT)
speed = GPIO.PWM(speedPin, speed_pwm_hz)
speed.start(speed_dc)
MAX_SPEED = 7.9
MIN_SPEED = 7.7

# Steering parameters
steeringPin = 19
steering_pwm_hz = 50
steering_dc = 7.5
steering_right_dc = 9.5
steering_left_dc = 5.5
GPIO.setup(steeringPin, GPIO.OUT)
steering = GPIO.PWM(steeringPin, steering_pwm_hz)
steering.start(steering_dc)
lastSteeringAngle = 90

# Video parameters
ESC_KEY_CODE = 27
VIDEO_DEVICE_IDX = 0
VIDEO_WIDTH = 160
VIDEO_HEIGHT = 120

# Optical encoder parameters
TIMESTAMP_PATH = "/sys/devices/platform/slay_device/speed"
prev_timestamp = -1

# PD variables
kp = 0.1  #started w 0.01
kd = kp * 0.05
lastTime = 0
lastError = 0


# arrays for making the final graphs
p_vals = []
d_vals = []
err_vals = []
speed_pwm = []
steer_pwm = []


# Steering control ############################################################
def set_steering(p, steering):
    p.ChangeDutyCycle(steering)


def reset_steering(p):
    p.ChangeDutyCycle(7.5)


def map_value(value, in_min, in_max, out_min, out_max):
    # Clamp input to avoid out-of-bound servo values
    value = max(min(value, in_max), in_min)
    return out_min + (float(value - in_min) / (in_max - in_min)) * (out_max - out_min)

###############################################################################

# ESC speed control ###########################################################
def set_esc(p, speed):
    global speed_dc
    speed_dc = speed

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


def accelerate():
    """
    Adjusts the speed parameters to increase velocity, if not at peak capacity.

    Returns:
        None: Provides update on velocity adjustments.
    """
    global speed_dc
    global MAX_SPEED
    
    if speed_dc < MAX_SPEED:
        speed_dc = min(speed_dc + 0.025, MAX_SPEED)  # Enhance speed
        set_esc(speed, speed_dc)
        print(f"Velocity increment: Now at {speed_dc}")
    else:
        print("Peak velocity achieved.")


def decelerate():
    """
    Modulates the speed parameters to reduce velocity, unless fully stopped.

    Returns:
        None: Delivers update on velocity reduction.
    """
    global speed_dc
    global MIN_SPEED

    if speed_dc > MIN_SPEED:
        speed_dc = max(speed_dc - 0.025, MIN_SPEED)  # Reduce speed
        set_esc(speed, speed_dc)
        print(f"Velocity decrement: Now at {speed_dc}")
    else:
        print("Complete standstill reached.")

###############################################################################

# Image processing functions ##################################################
def detect_edges(frame):
    # blue color detection
    # blurred = cv2.GaussianBlur(frame, (5,5), 0)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # cv2.imshow("HSV",hsv)
    lower_blue = np.array([100,120,50], dtype="uint8")
    upper_blue = np.array([140,255,255], dtype="uint8")
    mask = cv2.inRange(hsv,lower_blue,upper_blue) # this mask will filter out everything but blue

    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel, iterations=1)

    #Do GaussianBlur using mask instead of frame
    blurred = cv2.GaussianBlur(mask, (5,5), 0)

    cv2.imshow("mask",mask)

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
        # print("no line segment detected")
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
                # print("skipping vertical lines (slope = infinity)")
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


def get_steering_angle_custom(frame, lane_lines):
     height, width, _ = frame.shape
     mid = int(width / 2) #ideal center of frame
    
     if len(lane_lines) == 2: # if two lane lines are detected
        _, _, left_x2, _ = lane_lines[0][0] # extract left x2 from lane_lines array
        _, _, right_x2, _ = lane_lines[1][0] # extract right x2 from lane_lines array
        # mid = int(width / 2)
        # x_offset = (left_x2 + right_x2) / 2 - mid

        center_x = (left_x2 + right_x2) / 2 #get center between lanes
        x_offset = mid - center_x #reverse direction to go away from lane
        y_offset = int(height / 2)  

     elif len(lane_lines) == 1: # if only one line is detected
        x1, _, x2, _ = lane_lines[0][0]
        # x_offset = x2 - x1

        x_offset = mid - x2
        y_offset = int(height / 2)

     elif len(lane_lines) == 0: # if no line is detected
       # x_offset = 0
       # y_offset = int(height / 2)
       return 90 #keep straight
     angle_to_mid_radian = math.atan(x_offset / y_offset)
     angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)  
     steering_angle = angle_to_mid_deg + 90 

     return steering_angle

def get_steering_angle(frame, lane_lines):
    height, width, _ = frame.shape
    
    if len(lane_lines) == 2:
        _, _, left_x2, _ = lane_lines[0][0]
        _, _, right_x2, _ = lane_lines[1][0]
        mid = int(width / 2)
        x_offset = (left_x2 + right_x2) / 2 - mid
        y_offset = int(height / 2)
    
    elif len(lane_lines) == 1:
        x1, _, x2, _ = lane_lines[0][0]
        x_offset = x2 - x1
        y_offset = int(height / 2)
    
    elif len(lane_lines) == 0:
        x_offset = 0
        y_offset = int(height / 2)
    
    angle_to_mid_radian = math.atan(x_offset / y_offset)
    angle_to_mid_deg = int(angle_to_mid_radian * 180.0 / math.pi)
    steering_angle = angle_to_mid_deg + 90
    
    return steering_angle

def get_steering_center(center_x, frame_width):
    deviation = center_x - (frame_width / 2)

    #turn deviation into steering angle
    #tune scaling factor based on system's response
    steering_angle = 90 + (deviation * 0.1)

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

# Plotting functions ##########################################################
def plot_pd(p_vals, d_vals, error, show_img=False):
    # Plot the PD
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(p_vals))
    ax1.plot(t_ax, p_vals, '-', label="P values")
    ax1.plot(t_ax, d_vals, '-', label="D values")
    ax2 = ax1.twinx()
    ax2.plot(t_ax, error, '--r', label="Error")
    
    ax1.set_xlabel("Frames")
    ax1.set_ylabel("PD Value")
    ax2.set_ylim(-90, 90)
    ax2.set_ylabel("Error Value")
    
    plt.title("PD Values over time")
    fig.legend()
    fig.tight_layout()
    plt.savefig("pd_plot.png")
    
    if show_img:
        plt.show()
    plt.clf()


def plot_pwm(speed_pwms, turn_pwms, error, show_img=False):
    # Plot the PWM
    fig, ax1 = plt.subplots()
    t_ax = np.arange(len(speed_pwms))
    ax1.plot(t_ax, speed_pwms, '-', label="Speed PWM")
    ax1.plot(t_ax, turn_pwms, '-', label="Steering PWM")
    ax2 = ax1.twinx()
    ax2.plot(t_ax, error, '--r', label="Error")
    
    ax1.set_xlabel("Frames")
    ax1.set_ylabel("PWM Values")
    ax2.set_ylabel("Error Value")
    
    plt.title("PWM Values over time")
    fig.legend()
    plt.savefig("pwm_plot.png")
    
    if show_img:
        plt.show()
    plt.clf()

###############################################################################

# Set up GPIO
calibrate_esc(speed)

# Set up video stream
print("Initializing video stream...")
video = cv2.VideoCapture(VIDEO_DEVICE_IDX)
video.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
print("Video stream initialized.")
time.sleep(3)   # Allow time for camera window to open

# Start moving slowly
set_esc(speed, 7.72) #7.65

# The loop
while True:
    # print("speed_dc: ", speed_dc)
    ret, frame = video.read()
    cv2.imshow("original", frame)

    # Frame processing for steering
    edges = detect_edges(frame)
    roi = region_of_interest(edges)
    line_segments = detect_line_segments(roi)
    '''
    #5/1/25 Find the center between the lanes instead of tracking one
    if line_segments is not None and len(line_segments) >= 2:
        left_lane = min([line[0] for line in line_segments]) #left lane position
        right_lane = max([line[2] for line in line_segments]) #right lane position
        center_x = (left_lane + right_lane) / 2 #get center between lanes
        steering_angle = get_steering_center(center_x, frame.shape[1]) #use frame width
    else:
        steering_angle = lastSteeringAngle
    '''
    # lastSteeringAngle = steering_angle #save steering angle
    lane_lines = average_slope_intercept(frame,line_segments)
    lane_lines_image = display_lines(frame,lane_lines)
    steering_angle = get_steering_angle(frame, lane_lines)
    heading_image = display_heading_line(lane_lines_image,steering_angle)
    cv2.imshow("heading", heading_image)


    # TODO: Detect stop signs periodically
    

    # PD controller calculations
    now = time.time()
    dt = now - lastTime

    deviation = steering_angle - 90  # deviation from non-turned wheels

    # Error and PD adjustment
    error = -deviation
    base_turn = 0.0
    proportional = kp * error
    derivative = kd * (error - lastError) / dt
    
    # Data for graphing
    p_vals.append(proportional)
    d_vals.append(derivative)
    err_vals.append(error)
    
    # Steering adjustments
    turn_amt = base_turn + proportional + derivative

    # Adjust steering based on calculated turn amount
    turn_amt_mapped = map_value(
        turn_amt,
        in_min=-2.0,        # raw minimum
        in_max=2.0,         # raw maximum
        out_min=9,          # servo left
        out_max=6,          # servo right
    )
    set_steering(steering, turn_amt_mapped)
    # print(f"PD error: {turn_amt}, steering duty cycle: {turn_amt_mapped}")

    # print(f'raw turn amount: {turn_amt}')
    # if 7.3 < turn_amt < 7.6:
    #     turn_amt = 7.5
    # elif turn_amt > 9:
    #     turn_amt = 9.5
    # elif turn_amt < 6:
    #     turn_amt = 6
    # set_steering(steering, turn_amt)
    # print(f"Steering duty cycle: {turn_amt}")

    # Speed and steering updates for graphs
    steer_pwm.append(turn_amt)
    speed_pwm.append(speed_dc)

    # Read encoder data for speed adjustments
    time_diff_ms = 2
    with open("/sys/module/gpiod_driver/parameters/elapsed_ms", "r") as r:
        time_diff_ms = int(r.read())

    if (time_diff_ms != 0):
        print("time_diff: ", time_diff_ms)
    
    # Adjust speed based on encoder feedback
    if time_diff_ms >= 17:
        # Accelerate
        accelerate()
    elif 5 < time_diff_ms < 17:
        # Decelerate
        decelerate()
        
    # Update error values for PD control
    lastError = error
    lastTime = time.time()
    
    # Wait to exit
    key = cv2.waitKey(1)
    if key == ESC_KEY_CODE:
      break

# Cleanup
video.release()
cv2.destroyAllWindows()
reset_esc(speed)
reset_steering(steering)
GPIO.cleanup()

# Plot that shit
plot_pd(p_vals, d_vals, err_vals, True)
plot_pwm(speed_pwm, steer_pwm, err_vals, True)
