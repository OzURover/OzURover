#!/usr/bin/env python

import dynamixel
import rospy
import numpy as np
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from sensor_msgs.msg import Joy

def gripper(data):

	if(data.axes[6] < -0.1 or data.axes[6] > 0.1):
		print('gripper yaw')
		pub.publish(51+data.axes[6])

	if(data.axes[7] < -0.1 or data.axes[7] > 0.1):
		print('gripper pitch')
		pub.publish(61+data.axes[7])
	
	if(data.buttons[1] != 0):
		print('gripper claw close')
		pub2.publish(-200);
	elif(data.buttons[2] != 0):
		print('gripper claw open')
		pub2.publish(200);

def kinematics(data):
	#Gripper pitch
	dxl_angle = (data.data / np.pi)*2048*(2/1.5)

	if(data.data > 0):
		pub1.publish(2983 - dxl_angle)
	else:
		pub1.publish(2983 - dxl_angle)

def start():
	global pub, pub1, pub2
	rospy.init_node('Angle2Dxl')
	pub = rospy.Publisher('dxlinput', Float32, queue_size = 5)
	pub1 = rospy.Publisher('servokine', Float32, queue_size = 5)
	pub2 = rospy.Publisher('step_claw', Int32, queue_size = 5)
	rospy.Subscriber('joy', Joy, gripper)
	rospy.Subscriber('goal_pos4', Float32, kinematics)
	rospy.spin()

if __name__ == '__main__':
	start()
