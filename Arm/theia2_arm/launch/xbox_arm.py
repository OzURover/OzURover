#!/usr/bin/env python2.7
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy

def callback(data):
	#x
	if(data.buttons[6] == 1):
		if(data.axes[1] > 0.1 or data.axes[1] < -0.1):
			pubx.publish(data.axes[1])
			print(data.axes[1])
	#y
	if(data.buttons[5] == 1):
		if(data.axes[0] > 0.1 or data.axes[0] < -0.1):
			puby.publish(data.axes[0])
			print(data.axes[0])
	#z
	if(data.buttons[4] == 1):
		if(data.axes[1] > 0.1 or data.axes[1] < -0.1):
			pubz.publish(data.axes[1])
			print(data.axes[1])

def start():
	global puby, pubx, pubz
	rospy.init_node("xbox_arm", anonymous=True)
	pubx = rospy.Publisher("joyinputx", Float64, queue_size=10)
	puby = rospy.Publisher("joyinputy", Float64, queue_size=10)
	pubz = rospy.Publisher("joyinputz", Float64, queue_size=10)
	rospy.Subscriber("joy", Joy, callback)	
	rospy.spin()

if __name__ == '__main__':
	start()
