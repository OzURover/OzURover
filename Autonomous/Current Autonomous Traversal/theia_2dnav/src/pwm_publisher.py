#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped

def pwm_callback(rm_data, lm_data, rb_data, lb_data):
    rf_pwm = rm_data.point.x * 1.0 / 2
    lf_pwm = lm_data.point.x * 1.0 / 2
    arr = Float32MultiArray()
    arr.data = [lm_data.point.x, lf_data.point.x, lb_data.point.x, rm_data.point.x, rf_data.point.x, rb_data.point.x]
    wheel_pwm_publisher.publish(arr)

def setup():
	global wheel_pwm_publisher
	# Initialize the node.
	rospy.init_node('pwm_controller', anonymous = True);
	# Publisher for pwm.
	wheel_pwm_publisher = rospy.Publisher("joyinputx", Float32MultiArray, queue_size=5)
	# Subscriber for wheel encoders.
        right_mid_pwm_sub = message_filters.Subscriber("RM_pwm", PointStamped)
    	left_mid_pwm_sub = message_filters.Subscriber("LM_pwm", PointStamped)
    	right_back_pwm_sub = message_filters.Subscriber("RB_pwm", PointStamped)
    	left_back_pwm_sub = message_filters.Subscriber("LB_pwm", PointStamped)
	# Use synchonizer to get 4 encoder datas and bno data simultaneously.
    	time_synch = message_filters.ApproximateTimeSynchronizer([right_mid_pwm_sub, left_mid_pwm_sub, right_back_pwm_sub, left_back_pwm_sub], 10, 0.1, allow_headerless=True)
    	time_synch.registerCallback(pwm_callback)
	rospy.spin()

if __name__ == '__main__':
	setup()
