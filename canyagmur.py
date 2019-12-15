#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

def drive(data):
if(data.buttons[3]!=0):
pubx.publish(data.buttons[3])
elif(data.buttons[0]!=0):
    puby.publish(data.buttons)[0])
    else:
        pubx.publish(0)

        def start():
            global pubx,puby
            rospy.init_node("PWM2Arduino")
            pubx = rospy.Publisher("joyinputx", Float32, queue_size=2)
	          puby = rospy.Publisher("joyinputy", Float32, queue_size=2)
	          rospy.Subscriber("joy", Joy, drive)
	r             rospy.spin()

    if __name__ == '__main__':
	start()