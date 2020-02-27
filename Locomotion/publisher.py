#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy



def loco(data):
	if (data.buttons[5] == 1):
		if (data.axes[1] <=-0.1 or data.axes[1] > 0.1):
			pub.publish(data.axes[1])
			print(data.axes[1])
		else:
			pub.publish(0)
	if (data.buttons[4] == 1):
		if (data.axes[3] <=-0.1 or data.axes[3] > 0.1):
			pub.publish(data.axes[3]+20)
			print((data.axes[3]+20))
		else:
			pub.publish(0)

def start():
	global pub
	rospy.init_node("loco")
	pub = rospy.Publisher('loco', Float32, queue_size=10)
	rospy.Subscriber('joy', Joy, loco)
	rospy.spin()


if __name__ == '__main__':
	start()
