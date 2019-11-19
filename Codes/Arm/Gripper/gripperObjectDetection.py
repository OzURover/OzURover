#importing modules

import cv2   
import numpy as np

#capturing video through webcam
cap = cv2.VideoCapture("rtsp://192.168.0.11:554/user=admin_password=tlJwpbo6_channel=1_stream=0.sdp")
while(1):
	_, img = cap.read()
	    
	#converting frame(img i.e BGR) to HSV (hue-saturation-value)

	hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
	blue_upper=np.array([110,255,255],np.uint8)
	
	#defining the Range of yellow color
	yellow_lower=np.array([22,60,200],np.uint8)
	yellow_upper=np.array([60,255,255],np.uint8)

	green_lower = np.array([65,60,60],np.uint8)
	green_upper = np.array([80,255,255],np.uint8)

	#finding the range of red,blue and yellow color in the imageü
	yellow=cv2.inRange(hsv,yellow_lower,yellow_upper)
	green = cv2.inRange(hsv,green_lower,green_upper)
	
	#Morphological transformation, Dilation  	
	kernal = np.ones((5 ,5), "uint8")

	yellow=cv2.dilate(yellow,kernal)
	res2=cv2.bitwise_and(img, img, mask = yellow)

	green = cv2.dilate(green,kernal)
	res3 = cv2.bitwise_and(img, img, mask = yellow)

	(contours,hierarchy)=cv2.findContours(yellow,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	for pic, contour in enumerate(contours):
		area = cv2.contourArea(contour)
		if(area>300):
			x,y,w,h = cv2.boundingRect(contour)	
			img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
			cv2.putText(img,"Cache",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0))

	(contours,hierarchy)=cv2.findContours(green,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
	for pic, contour in enumerate(contours):
		area = cv2.contourArea(contour)
		if(area>300):
			x,y,w,h = cv2.boundingRect(contour)	
			img = cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
			cv2.putText(img,"Cache",(x,y),cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0,255,0))  
            
           
    	#cv2.imshow("Redcolour",red)
	cv2.imshow("Color Tracking",img)
    	#cv2.imshow("red",res)
	if cv2.waitKey(10) & 0xFF == ord('q'):
    		cap.release()
    		cv2.destroyAllWindows()
    		break  
          

    
