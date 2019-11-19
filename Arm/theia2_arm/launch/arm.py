#!/usr/bin/env python2.7
import rospy
import time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Header
from sensor_msgs.msg import Joy
	
joint_state1 = JointState()

q1 = 0
q2 = 0
q3 = 0
q4 = 0
q5 = 0

def chatter1(message):
	global q1, q2, q3, q4, q5 
	q1 = message.data*3.14
	joint_state1.position=[q1,q2,q3,q4,q5]

def chatter2(message):
	global q1, q2, q3, q4, q5 
	q2 = message.data*3.14/2
	joint_state1.position=[q1,q2,q3,q4,q5]

def chatter3(message):
	global q1, q2, q3, q4, q5 
	q3 = message.data*3.14
	joint_state1.position=[q1,q2,q3,q4,q5]
	
def talker():
	global q1, q2, q3, q4, q5
	rospy.init_node('joint_state_publisher')
	rospy.Subscriber('joyinputx',Float64,chatter1)
	rospy.Subscriber('joyinputy',Float64,chatter2)
	rospy.Subscriber('joyinputz',Float64,chatter3)
	pub = rospy.Publisher('joint_states', JointState, queue_size=10)
	rate = rospy.Rate(10)
	
	joint_state1.header = Header()
	joint_state1.name=['base_link__link_01','link_01__link_02','link_02__link_03','link_03__endeffector_y','endeffector_y__endeffector']

	joint_state1.velocity=[]
	joint_state1.effort=[]

	while not rospy.is_shutdown():
		joint_state1.header.stamp = rospy.Time.now()
		pub.publish(joint_state1)
		rate.sleep()

if __name__ == '__main__':	
	try:
		talker()
	except rospy.ROSInterruptException:
		pass
