#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy


def drive(data):
	#j0 a button on xbox
	if(data.buttons[5] != 0):
		if(data.axes[0] < -0.1 or data.axes[0] > 0.1):
			pubJ0.publish(data.axes[1])
	#j1 b button on xbox
	elif(data.buttons[4] != 0):
		if(data.axes[1] < -0.1 or data.axes[1] > 0.1):
			pubJ1.publish(data.axes[1])	
	#j2 y button on xbox
	elif(data.buttons[6] != 0):
		if(data.axes[1] < -0.1 or data.axes[1] > 0.1):
			pubJ2.publish(data.axes[1])
	else:
		pubJ0.publish(0)
		pubJ1.publish(0)
		pubJ2.publish(0)


def start():
	global pubJ0, pubJ1, pubJ2
	rospy.init_node("Manuelarm")
	pubJ0 = rospy.Publisher("joyinputj0", Float32, queue_size=2)
	pubJ1 = rospy.Publisher("joyinputj1", Float32, queue_size=2)
	pubJ2 = rospy.Publisher("joyinputj2", Float32, queue_size=2)
	rospy.Subscriber("joy", Joy, drive)
	rospy.spin()

if __name__ == '__main__':
	start()
