#!/usr/bin/env python
import cv2
import numpy as np
from matplotlib import pyplot as plt
import rospy
from rospy.exceptions import ROSException
from sensor_msgs.msg import Image


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
   # image_bgr = cv2.imread('/home/salih/Desktop/Mars.jpg', cv2.IMREAD_COLOR)
    image_bgr = cv2.imread(Image, cv2.IMREAD_COLOR)
    channels = cv2.mean(image_bgr)

    # Swap blue and red values (making it RGB, not BGR)
    observation = np.array([(channels[2], channels[1], channels[0])])
    print(observation)

    mean_of_Red   = channels[2]
    mean_of_Green = channels[1]
    mean_of_Blue  = channels[0]

    # red percent will be used for forecasting pH
    percent_of_Red = mean_of_Red * 100 /(mean_of_Red+mean_of_Blue+mean_of_Green)
    print(str(percent_of_Red)+" %")

    # equation taken from : http://przyrbwn.icm.edu.pl/APP/PDF/132/app132z3-IIp086.pdf & https://core.ac.uk/download/pdf/158352623.pdf

    # y = 0.0956 * x + 4.2722 ==> y: pH and x: Red Values (%)

    #graph('0.0956*x+4.2722', range(0, 100),percent_of_Red)

    rospy.loginfo("Predicted pH: "+ str(0.0956 * percent_of_Red + 4.2722 ))


def main():
  rospy.init_node('marsyard_pH_forecasting', anonymous=True)
  rospy.loginfo("marsyard_pH_forecasting node initiliazed !")
  image_sub = rospy.Subscriber("/zed2/right_raw/image_raw_color",Image, image_processor)
  try:
    rospy.spin()
  except:
    print("Shutting down")
  cv2.destroyAllWindows()



#plt.imshow(observation), plt.axis("off")
#plt.show()


if __name__ == "__main__":
    main()