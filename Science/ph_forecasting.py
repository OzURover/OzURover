#!/usr/bin/env python
import sys
import cv2
import numpy as np
from matplotlib import pyplot as plt
from numpy.lib.type_check import imag
import rospy
from rospy.exceptions import ROSException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from datetime import datetime
from pynput.keyboard import Listener  #will be listening keyboard

#This code will take 1 argument , while launching: python ph_forecasting.py -show ==> Recommended for observing what's going on
# If you do not give any arg. , then it doesn't show any GUI. ==> Recommended for Competetion time
intialTracbarVals = [153,498,160,670]

p_Pressed = False
def keyPressed(key):
    global p_Pressed
    if(key.char == ('p') ):
        p_Pressed = True
        print("p_Pressed convertod ==> True ")

listener = Listener(on_press=keyPressed)
listener.start()


def graph(formula, x_range,percent_of_Red):

    x = np.array(x_range)  
    y = eval(formula)
    
        # naming the x axis
    plt.xlabel('Red (%)')
    # naming the y axis
    plt.ylabel('pH Values')
    plt.plot(x, y)  
    plt.plot(percent_of_Red,0.0956 * percent_of_Red + 4.2722 ,'ro') 
    plt.annotate("Predicted pH: "+  str(0.0956 * percent_of_Red + 4.2722), (percent_of_Red,0.0956 * percent_of_Red + 4.2722 ))
    plt.show()


def image_processor(Image):

    # Converting from Image to open cv image type:
    bridge = CvBridge()

    try:
      img_cv2 = bridge.imgmsg_to_cv2(Image, "bgr8")
    except CvBridgeError as e:
      print(e)

    # image wrapping
    img_wrapped = warpImg(img_cv2,valTrackbars(),1000,800) 
    # image_bgr = cv2.imread('/home/salih/Desktop/Mars.jpg', cv2.IMREAD_COLOR)


    channels = cv2.mean(img_wrapped)
    # Swap blue and red values (making it RGB, not BGR)
    observation = np.array([(channels[2], channels[1], channels[0])])
    #print(observation)

    mean_of_Red   = channels[2]
    mean_of_Green = channels[1]
    mean_of_Blue  = channels[0]

    # red percent will be used for forecasting pH
    percent_of_Red = mean_of_Red * 100 /(mean_of_Red+mean_of_Blue+mean_of_Green)
   # print(str(percent_of_Red)+" %")

    # equation taken from : http://przyrbwn.icm.edu.pl/APP/PDF/132/app132z3-IIp086.pdf & https://core.ac.uk/download/pdf/158352623.pdf

    # y = 0.0956 * x + 4.2722 ==> y: pH and x: Red Values (%)

    #graph('0.0956*x+4.2722', range(0, 100),percent_of_Red)
    predicted_pH = 0.0956 * percent_of_Red + 4.2722
    
   
    # Text Settings:
        # font
    font = cv2.FONT_HERSHEY_SIMPLEX
      
    # org
    org = (250, 500)
      
    # fontScale
    fontScale = 1
      
    # Blue color in BGR
    color = (255, 0, 0)
      
    # Line thickness of 2 px
    thickness = 2
      
    # Using cv2.putText() method
    image = cv2.putText(img_cv2, 'Predicted pH: '+str(predicted_pH), org, font, 
                      fontScale, color, thickness, cv2.LINE_AA)
    img_drawn = drawPoints(image,valTrackbars())
    if(len(sys.argv) > 1 and sys.argv[1] in ['-show']):
      
      cv2.imshow("Image",img_drawn)
      cv2.waitKey(1)
      
    else:
      global p_Pressed
      rospy.loginfo("Predicted pH: "+ str(predicted_pH ))
      if (p_Pressed):
        cv2.imwrite("./savedImage{}.jpg".format(datetime.now()),img_drawn)    #Need to be checked if location is fine ? 
        print("pH Forecasted image saved SUCCESSFULLY!")
        p_Pressed = False




#plt.imshow(observation), plt.axis("off")
#plt.show()


def warpImg(img, points, w, h, inv=False):
    pts1 = np.float32(points)
    pts2 = np.float32([[0, 0], [w, 0], [0, h], [w, h]])
    if inv:
        matrix = cv2.getPerspectiveTransform(pts2, pts1)
    else:
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
    imgWarp = cv2.warpPerspective(img, matrix, (w, h))
    return imgWarp

def nothing():
  pass
  

def initializeTrackbars(intialTracbarVals, wT=1000, hT=800):
    cv2.namedWindow("Trackbars")
    cv2.resizeWindow("Trackbars", 360, 240)
    cv2.createTrackbar("Width Top", "Trackbars", intialTracbarVals[0], wT // 2, nothing)
    cv2.createTrackbar("Height Top", "Trackbars", intialTracbarVals[1], hT, nothing)
    cv2.createTrackbar("Width Bottom", "Trackbars", intialTracbarVals[2], wT // 2, nothing)
    cv2.createTrackbar("Height Bottom", "Trackbars", intialTracbarVals[3], hT, nothing)


def valTrackbars(wT=1000, hT=600):

  if len(sys.argv) == 1:
    
    widthTop = intialTracbarVals[0]
    heightTop = intialTracbarVals[1]
    widthBottom = intialTracbarVals[2]
    heightBottom = intialTracbarVals[3]
  
  
  elif(sys.argv[1] in ['-show']):
     widthTop = cv2.getTrackbarPos("Width Top", "Trackbars")
     heightTop = cv2.getTrackbarPos("Height Top", "Trackbars")
     widthBottom = cv2.getTrackbarPos("Width Bottom", "Trackbars")
     heightBottom = cv2.getTrackbarPos("Height Bottom", "Trackbars")
     
  
  points = np.float32([(widthTop, heightTop), (wT - widthTop, heightTop),
                    (widthBottom, heightBottom), (wT - widthBottom, heightBottom)])
  return points


         
    


def drawPoints(img, points):
    for x in range(4):
        cv2.circle(img, (int(points[x][0]), int(points[x][1])), 15, (0, 0, 255), cv2.FILLED)
    return img



def main():
  rospy.init_node('marsyard_pH_forecasting', anonymous=True)
  rospy.loginfo("marsyard_pH_forecasting node initiliazed !")
  image_sub = rospy.Subscriber("/zed2/right/image_rect_color",Image, image_processor)
  try:
    rospy.spin()
  except:
    print("Shutting down")
  cv2.destroyAllWindows()





if __name__ == "__main__":

  if len(sys.argv) ==1:
    pass
  elif(sys.argv[1] in ['-show']):
    initializeTrackbars(intialTracbarVals)
  
  main()