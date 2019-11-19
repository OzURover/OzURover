#!/usr/bin/env python

# import the necessary packages for processing.
import numpy as np
import imutils
import cv2
import sys

# import ros packages
import rospy
from std_msgs.msg import String

camera = cv2.VideoCapture("rtsp://admin:@192.168.137.93/user=admin_password=tlJwpbo6_channel=1_stream=0.sdp")

def detecter():
    pub = rospy.Publisher('detect_info', String, queue_size=10)
    rospy.init_node('ball_tracker')

    greenLower = np.array([29, 100, 100])
    greenUpper = np.array([64, 255, 255])


    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        (grabbed, frame) = camera.read()

        frame = imutils.resize(frame, width=600)
        blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, greenLower, greenUpper)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)

        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None

        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
            M = cv2.moments(largest_contour)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            if radius > 2:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                pub.publish("Found a ball")
        else:
            pub.publish("Found nothing")

        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

    	# if the 'q' key is pressed, stop the loop
    	if key == ord("q"):
    		break


def main(args):
    detecter()

if __name__ == '__main__':
    main(sys.argv)
