#!/usr/bin/env python
import rospy
import message_filters
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PointStamped, Point

# Initial pwm variables for every single motor.
rm_pwm_value = 0.3
lm_pwm_value = 0.3
rb_pwm_value = 0.3
lb_pwm_value = 0.3
# Maximum pwm value.
max_pwm = 1.0
# Maximum velocity threshold.
max_threshold_velocity = 0.0065

def autonomy_callback(RM_info, LM_info, RB_info, LB_info):
    global rf_pwm_value, lf_pwm_value, rm_pwm_value, lm_pwm_value, rb_pwm_value, lb_pwm_value

    # Put velocities in a list.
    RM_velocity = -RM_info.point.y
    LM_velocity = LM_info.point.y
    RB_velocity = -RB_info.point.y
    LB_velocity = LB_info.point.y

    print("Velocities: ", RM_velocity, LM_velocity, RB_velocity, LB_velocity)
    # Put velocities in a list.
    velocities = [RM_velocity, LM_velocity, RB_velocity, LB_velocity]

    if max(velocities) > max_threshold_velocity:
        current_max_velocity = max_threshold_velocity
    else:
        current_max_velocity = max(velocities)
    print("Current max velocity:", current_max_velocity)
    # Increase pwm if it is lower than max velocity.
    if RM_velocity < current_max_velocity:
        rm_pwm_value = rm_pwm_value * 1.01
        if rm_pwm_value > max_pwm:
            rm_pwm_value = max_pwm
    else:
	rm_pwm_value = rm_pwm_value * 0.99
    if LM_velocity < current_max_velocity:
        lm_pwm_value = lm_pwm_value * 1.02
        if lm_pwm_value > max_pwm:
            lm_pwm_value = max_pwm
    else:
    	lm_pwm_value = lm_pwm_value * 0.99
    if RB_velocity < current_max_velocity:
        rb_pwm_value = rb_pwm_value * 1.02
        if rb_pwm_value > max_pwm:
            rb_pwm_value = max_pwm
    else:
	rb_pwm_value = rb_pwm_value * 0.99
    if LB_velocity < current_max_velocity:
        lb_pwm_value = lb_pwm_value * 1.02
        if lb_pwm_value > max_pwm:
            lb_pwm_value = max_pwm
    else:
	lb_pwm_value = lb_pwm_value * 0.99

    lf_pwm_value = 1
    rf_pwm_value = 0.85

    arr = Float32MultiArray()
    arr.data = [lf_pwm_value, lm_pwm_value, lb_pwm_value, rf_pwm_value, rm_pwm_value, rb_pwm_value]
    forward_pwm_publisher.publish(arr)


def setup():
	global forward_pwm_publisher
	# Initialize the node.
	rospy.init_node('autonomy_controller', anonymous = True);
	# Encoder data subscribers, using time synch in order to receive data simultaneously.
	right_mid_encoder_sub = message_filters.Subscriber("RM_info", PointStamped)
	left_mid_encoder_sub = message_filters.Subscriber("LM_info", PointStamped)
	right_back_encoder_sub = message_filters.Subscriber("RB_info", PointStamped)
	left_back_encoder_sub = message_filters.Subscriber("LB_info", PointStamped)
	time_synch = message_filters.ApproximateTimeSynchronizer([right_mid_encoder_sub, left_mid_encoder_sub, right_back_encoder_sub, left_back_encoder_sub], 10, 0.1)
	time_synch.registerCallback(autonomy_callback)
	# Pwm publishers for every single motor.
	forward_pwm_publisher = rospy.Publisher("joyinputx", Float32MultiArray, queue_size=5)
	rospy.spin()


if __name__ == '__main__':
	setup()
