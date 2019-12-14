#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy


def gas(data):
	if(data.buttons[4] != 0):
		
		pubx.publish(1)
	elif(data.buttons[5] != 0):
		
		pubx.publish(-1)
		
	else:
		pubx.publish(0)


def begin():
	global pubx
	rospy.init_node("PWM2Arduino")
	pubx = rospy.Publisher("joyinputx", Float32, queue_size=1)
	rospy.Subscriber("joy", Joy, gas)
	rospy.spin()



if __name__ == '__main__':
	begin()
