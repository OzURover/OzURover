#!/usr/bin/env python
import rospy
import message_filters
from geometry_msg import Point, PointStamped
from std_msgs.msg import Float32, Float32MultiArray

def compund_pwm(lb_data, lm_data, rb_data, rm_data):
	pwm_arr = Float32Float32MultiArray()
	arr.data = [rm_data.point.x, rm_data.point.x * 1.0 / 2, rb_data.point.x, lm_data.point.x, lm_data.point.x * 1.0 / 2, lb_data.point.x]
	pwm_publisher.publish(arr)

def setup():
	global pwm_publisher
	# Initialize the node.
	rospy.init_node('pwm_compounder', anonymous = True);
	# Publisher for pwm.
	pwm_publisher = rospy.Publisher("mega_diff_drive", Float32MultiArray, queue_size=10)
	# Pwm subscribers.
	lb_pwm_subscriber = message_filters.Subscriber("lb_motor_cmd", PointStamped)
	lm_pwm_subscriber = message_filters.Subscriber("lm_motor_cmd", PointStamped)
    rb_pwm_subscriber = message_filters.Subscriber("rb_motor_cmd", PointStamped)
	rm_pwm_subscriber = message_filters.Subscriber("rm_motor_cmd", PointStamped)
    # Time synchonizer for both pwm values.
    time_synch = message_filters.ApproximateTimeSynchronizer([lb_pwm_subscriber, lm_pwm_subscriber, rb_pwm_subscriber, rm_pwm_subscriber], 10, 0.1, allow_headerless=True)
    time_synch.registerCallback(compund_pwm)
	rospy.spin()

if __name__ == '__main__':
	setup()
