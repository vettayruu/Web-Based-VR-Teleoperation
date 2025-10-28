import cv2
import numpy
import time

# Open the ZED camera
cap = cv2.VideoCapture(6)
if cap.isOpened() == 0:
    exit(-1)

# Set the video resolution to HD720 (2560*720)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

while True :
    # Get a new frame from camera
    retval, frame = cap.read()
    # Extract left and right images from side-by-side
    # Display images
    # cv2.imshow("frame", frame)
    cv2.imshow("left", frame)
    if cv2.waitKey(1) >= 0 :
        break

exit(0)