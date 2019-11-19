#!/usr/bin/env python
import rospy
import message_filters
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PointStamped, Point

# Initial pwm variables for every single motor.
rm_pwm_value = 0.7
lm_pwm_value = 0.7
rb_pwm_value = 0.7
lb_pwm_value = 0.7
# Maximum pwm value.
max_pwm = 1
# Minimum pwm value.
min_pwm = 0.1
# Maximum velocity threshold.
max_threshold_velocity = 50

def autonomy_callback(RM_info, LM_info, RB_info, LB_info):
    global rf_pwm_value, lf_pwm_value, rm_pwm_value, lm_pwm_value, rb_pwm_value, lb_pwm_value
    # Put velocities in a list.
    RM_velocity = RM_info.point.y
    LM_velocity = LM_info.point.y
    RB_velocity = RB_info.point.y
    LB_velocity = LB_info.point.y

    mean_velocity = (RM_velocity + LM_velocity + RB_velocity + LB_velocity) / 4

    if mean_velocity > max_threshold_velocity:
        target_velocity = max_threshold_velocity
    else:
        target_velocity = mean_velocity
    # Increase pwm if it is lower than max velocity.
    if RM_velocity < target_velocity:
        rm_pwm_value = rm_pwm_value * 1.2
        if rm_pwm_value > max_pwm:
            rm_pwm_value = max_pwm
    else:
        rm_pwm_value = rm_pwm_value * 0.8
        if rm_pwm_value < min_pwm:
            rm_pwm_value = min_pwm
    if LM_velocity < target_velocity:
        lm_pwm_value = lm_pwm_value * 1.2
        if lm_pwm_value > max_pwm:
            lm_pwm_value = max_pwm
    else:
        lm_pwm_value = lm_pwm_value * 0.8
        if lm_pwm_value < min_pwm:
            lm_pwm_value = min_pwm
    if RB_velocity < target_velocity:
        rb_pwm_value = rb_pwm_value * 1.2
        if rb_pwm_value > max_pwm:
            rb_pwm_value = max_pwm
    else:
        rb_pwm_value = rb_pwm_value * 0.8
        if rb_pwm_value < min_pwm:
            rb_pwm_value = min_pwm
    if LB_velocity < target_velocity:
        lb_pwm_value = lb_pwm_value * 1.2
        if lb_pwm_value > max_pwm:
            lb_pwm_value = max_pwm
    else:
        lb_pwm_value = lb_pwm_value * 0.8
        if lb_pwm_value < min_pwm:
            lb_pwm_value = min_pwm

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
