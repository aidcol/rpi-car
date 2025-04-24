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

import cv2


# Video parameters
VIDEO_WIDTH = 160
VIDEO_HEIGHT = 120

# Set up video stream
video = cv2.VideoCapture(2)
video.set(cv2.CAP_PROP_FRAME_WIDTH, VIDEO_WIDTH)
video.set(cv2.CAP_PROP_FRAME_HEIGHT, VIDEO_HEIGHT)
time.sleep(1)


# The loop
while True:
  ret, frame = video.read()
  frame = cv2.flip(frame, -1)
  cv2.imshow("original", frame)

  key = cv2.waitKey(1)
  if key == 27:
    break

video.release()
cv2.destroyAllWindows()
