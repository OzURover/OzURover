#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

def drive(data):
	if(data.buttons[4] != 0):
		print(data.axes[4])
		if(data.axes[4] < -0.1 or data.axes[4] > 0.1):
			pub1.publish(data.axes[4])


def start():
	global pub1
	rospy.init_node("PWM2Arduino")
	pub1 = rospy.Publisher("joyinputy", Float32, queue_size=5)
	while not rospy.is_shutdown():
		rospy.Subscriber("joy", Joy, drive)
		rospy.spin()

if __name__ == '__main__':
	start()
