import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy

import ikpy
import numpy as np
from ikpy import plot_utils
import matplotlib.pyplot as plt
import time

target_vector = [0,0,0] 
target_frame = np.eye(4)


def ik(data):
	global pub0, pub1, pub2
	my_chain = ikpy.chain.Chain.from_urdf_file("th2.URDF")

	#x
	if(data.buttons[6] == 1):
		if(data.axes[1] > 0.1 or data.axes[1] < -0.1):
			target_vector[0] = target_vector[0] + data.axes[1]*0.05	
			target_frame[:3, 3] = target_vector
			fk = my_chain.inverse_kinematics(target_frame)
			real_frame = my_chain.forward_kinematics(my_chain.inverse_kinematics(target_frame))
			print("Current Position:" ,real_frame)
			pub0.publish(fk[1])
	
	#y	
	if(data.buttons[5] == 1):
		if(data.axes[0] > 0.1 or data.axes[0] < -0.1):
			target_vector[1] = target_vector[1] + data.axes[0]*0.05
			target_frame[:3, 3] = target_vector
			fk = my_chain.inverse_kinematics(target_frame)
			real_frame = my_chain.forward_kinematics(my_chain.inverse_kinematics(target_frame))
			print("Current Position:" ,real_frame)
			pub1.publish(fk[2])

	#z
	if(data.buttons[4] == 1):
		if(data.axes[1] > 0.1 or data.axes[1] < -0.1):
			target_vector[2] = target_vector[2] + data.axes[1]*0.05
			target_frame[:3, 3] = target_vector
			fk = my_chain.inverse_kinematics(target_frame)
			real_frame = my_chain.forward_kinematics(my_chain.inverse_kinematics(target_frame))
			print("Current Position:" ,real_frame)
			pub2.publish(fk[3])


def start():
	rospy.init_node('IK_Node')
	rospy.Subscriber("joy", Joy, ik)
	pub0 = rospy.Publisher("target0", Float32, queue_size=10)
	pub1 = rospy.Publisher("target1", Float32, queue_size=10)	
	pub2 = rospy.Publisher("target2", Float32, queue_size=10)
	rospy.spin()
if __name__ == '__main__':
	start()
