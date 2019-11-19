#!/usr/bin/env python2.7
import rospy
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Header

def talker():
	rospy.init_node('joint_state_publisher')
	pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	rate = rospy.Rate(10)

	joint_state1 = JointState()
	joint_state1.header = Header()
	joint_state1.name=['base_link__link_01','link_01__link_02','link_02__link_03','link_03__endeffector_y','endeffector_y__endeffector']
	joint_state1.position=[2.2,0.6,1.4,1.0,1.0]
	joint_state1.velocity=[]
	joint_state1.effort=[]
	q1 = 0
	q2 = 0
	q3 = 0
	q4 = 0
	q5 = 0

	while not rospy.is_shutdown():
		joint_state1.header.stamp = rospy.Time.now()
		q1 = q1+0.1
		q2 = q2+0.1
		q3 = q3+0.1
		joint_state1.position=[q1,q2,q3,q4,q5]
		pub.publish(joint_state1)
		rate.sleep()
		

if __name__ == '__main__':	
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
