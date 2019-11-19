#!/usr/bin/env python

# import the necessary packages
import numpy as np
import cv2
import imutils
import time

greenLower = np.array([29, 100, 100])
greenUpper = np.array([64, 255, 255])

def find_marker(image):
    global mask
    frame = imutils.resize(image, width=600)
    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    mask = cv2.inRange(hsv, greenLower, greenUpper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None
    largest_contour = max(contours, key=cv2.contourArea)
    ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
    # compute the bounding box of the of the paper region and return it
    return radius

def distance_to_camera(knownWidth, focalLength, perWidth):
	# compute and return the distance from the maker to the camera
	return (knownWidth * focalLength) / perWidth

# initialize the known distance from the camera to the object, which
# in this case is 30 cms
KNOWN_DISTANCE = 40.0

# initialize the known object width, which in this case, the piece of
# paper is 11 cms wide
KNOWN_WIDTH = 6.0

# load the furst image that contains an object that is KNOWN TO BE 2 feet
# from our camera, then find the paper marker in the image, and initialize
# the focal length
image = cv2.imread("../images/ball-40-cm.png")
radius = find_marker(image)
focalLength = (radius * KNOWN_DISTANCE) / KNOWN_WIDTH
print("Focal Length:", focalLength)

image = cv2.imread("../images/ball-100-cm.png")
radius = find_marker(image)
cms = distance_to_camera(KNOWN_WIDTH, focalLength, radius)

# draw a bounding box around the image and display it
cv2.putText(image, "%.2fcm" % (cms),
    (image.shape[1] - 200, image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
    2.0, (0, 255, 0), 3)
cv2.imshow("image", image)
cv2.imshow("mask", mask)
cv2.waitKey(0)
