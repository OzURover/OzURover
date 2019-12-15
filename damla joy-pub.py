#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
		
def drive(data):
	if(data.buttons[5] != 1):
		if(data.axes[4] < 0): 	
			puby.publish(data.axes[4])
		elif(data.axes[4] > 0):
			puby.publish(data.axes[4])		
    else:
        puby.publish(0)
def start():
	global puby
	rospy.init_node("PWMArduino")
	puby = rospy.Publisher("joyinputy", Float32, queue_size=2)
	rospy.Subscriber("joy", Joy, drive)
	rospy.spin()

if __name__ == '__main__':
	start()