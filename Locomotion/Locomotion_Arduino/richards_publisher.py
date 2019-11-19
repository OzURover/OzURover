#!/usr/bin/env python

import rospy
import time
import math
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

# Logitech
halt = False
axis_state = True
power_multiplier = 1

def drive(data):
	global axis_state, halt, power_multiplier

	power_multiplier = (data.axes[3] + 1) / 2
	x = data.axes[1] * power_multiplier
	y = data.axes[0] * power_multiplier
	ang = abs((math.atan(x/y) * 180 / math.pi))

	if (ang > 45):
		axis_state = True
	else:
		axis_state = False
		
	if (data.buttons[2] == 1): # Button can change
		halt = not halt
		time.sleep(0.3)
		print(halt)

	if (not halt): # Safety measurement
		if (axis_state):
			puby.publish(0)
			pubx.publish(x)
		else:
			pubx.publish(0)
			puby.publish(y)
	else:
		pubx.publish(0)
		puby.publish(0)

def start():
	global pubx, puby
	rospy.init_node("PWM2Arduino")
	pubx = rospy.Publisher("joyinputx", Float32, queue_size=2)
	puby = rospy.Publisher("joyinputy", Float32, queue_size=2)
	rospy.Subscriber("joy", Joy, drive)
	rospy.spin()

if __name__ == '__main__':
	start()

