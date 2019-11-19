#!/usr/bin/env python

import rospy
import time
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
		
# Legacy Xbox implementation
def drive(data):
	if(data.buttons[4] != 0):
		if(data.axes[4] < -0.1 or data.axes[4] > 0.1):
			pubx.publish(data.axes[4])
	elif(data.buttons[5] != 0):
		if(data.axes[2] < 1):
			#to turn left 	
			puby.publish(1.0 - data.axes[2])
		elif(data.axes[5] < 1):
			#to turn right
			puby.publish(-1 * (1.0 - data.axes[5]))		
	else:
		pubx.publish(0)

def start():
	global pubx, puby
	rospy.init_node("PWM2Arduino")
	pubx = rospy.Publisher("joyinputx", Float32, queue_size=2)
	puby = rospy.Publisher("joyinputy", Float32, queue_size=2)
	rospy.Subscriber("joy", Joy, drive)
	rospy.spin()

if __name__ == '__main__':
	start()
