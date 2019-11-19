#!/usr/bin/env python

# import the necessary packages for processing.
import numpy as np
import imutils
import cv2
import sys

# import ros packages
import rospy
from std_msgs.msg import String
from std_msgs.msg import Float32

camera = cv2.VideoCapture("rtsp://192.168.88.90:554/user=admin&password=&channel=1&stream=0.sdp?")
ball_found = False
greenLower = np.array([29, 100, 100])
greenUpper = np.array([64, 255, 255])

def detect():
    global rotate_publisher
    rotate_publisher = rospy.Publisher('joyinputx', Float32, queue_size=5)
    rospy.init_node('ball_tracker_to_pwm')

    while not rospy.is_shutdown():
        (grabbed, frame) = camera.read()

        frame = imutils.resize(frame, width=600)
        #blurred = cv2.GaussianBlur(frame, (11, 11), 0)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

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

            if radius > 5:
                cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 255), 2)
                # distance = distance_to_camera(focalLength, radius)
                # cv2.putText(frame, "%.2fcm" % (distance),
                #     (frame.shape[1] - 200, frame.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX,
                #     2.0, (0, 255, 0), 3)
                rotate_publisher.publish(1) # Turn right
            else:
                rotate_publisher.publish(0) # Stay
        else:
            rotate_publisher.publish(0) # Stay

        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
        key = cv2.waitKey(1) & 0xFF
        
    	# if the 'q' key is pressed, stop the loop
        if key == ord("q"):
    		break

def main(args):
    detect()

if __name__ == '__main__':
    main(sys.argv)
