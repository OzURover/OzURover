#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy


def drive(data):

#j2 
	if(data.buttons[0] != 0):
		if(data.axes[1] < -0.1 or data.axes[1] > 0.1):
			pubJ2.publish(data.axes[1])
	else:
		pubJ2.publish(0)
	
	

def start():
	global pubJ2
	rospy.init_node("Manuelarm2")
	pubJ2 = rospy.Publisher("joyinputj2", Float32, queue_size=2)
	rospy.Subscriber("joy2", Joy, drive)
	rospy.spin()

if __name__ == '__main__':
	start()
