#!/usr/bin/env python

import cv2 as cv
from time import gmtime, strftime
import numpy as np
import math

flag=1

def capture():
	global flag
	while(flag):
		vcap = cv.VideoCapture("rtsp://192.168.88.90:554/user=admin&password=&channel=1&stream=0.sdp?")
		ret, frame = vcap.read()

		time=strftime("%d-%m-%Y | %H.%M.%S", gmtime())
		time1=strftime("%Y-%m-%d-%H-%M-%S", gmtime())
		img_new=cv.copyMakeBorder(frame,0,60,0,0,cv.BORDER_CONSTANT,value=[0,0,0])
		cv.putText(img_new, "Ozyegin University Rover Team - Theia", (0,740), cv.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,255),1)

		cv.putText(img_new, "-- Capture Time: "+time, (0,770), cv.FONT_HERSHEY_SIMPLEX, 0.75, (255,255,255),1)

		cv.imwrite('../images/ball-100-cm.png', img_new)

		print "Done!"

		flag=0

if __name__ == '__main__':
	capture()
