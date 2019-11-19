#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy


def drive(data):

#j1 
	if(data.buttons[0] != 0):
		if(data.axes[1] < -0.1 or data.axes[1] > 0.1):
			pubJ1.publish(data.axes[1])
	
	#j0 
	elif(data.buttons[1] != 0):
		if(data.axes[2] < -0.1 or data.axes[2] > 0.1):
			pubJ0.publish(data.axes[2])

	else:
		pubJ0.publish(0)
		pubJ1.publish(0)


def start():
	global pubJ0, pubJ1
	rospy.init_node("Manuelarm")
	pubJ0 = rospy.Publisher("joyinputj0", Float32, queue_size=2)
	pubJ1 = rospy.Publisher("joyinputj1", Float32, queue_size=2)
	rospy.Subscriber("joy1", Joy, drive)
	rospy.spin()

if __name__ == '__main__':
	start()
