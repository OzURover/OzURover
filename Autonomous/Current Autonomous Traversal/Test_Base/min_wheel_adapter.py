
# !/usr/bin/env python
import rospy
import message_filters
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PointStamped, Point

# Initial pwm variables for every single motor.
rm_pwm_value = 1.0
lm_pwm_value = 1.0
rb_pwm_value = 1.0
lb_pwm_value = 1.0
# Max pwm value.
max_pwm = 1.0
# Min pwm value.
min_pwm = 0.2
# Pwm factor.
pwm_factor = 0.00001
# Acceptable error for velocity.
velocity_error = 0.000005

def autonomy_callback(RM_info, LM_info, RB_info, LB_info):
    global rf_pwm_value, lf_pwm_value, rm_pwm_value, lm_pwm_value, rb_pwm_value, lb_pwm_value

    # Put velocities in a list.
    RM_velocity = - RM_info.point.y
    LM_velocity = LM_info.point.y
    RB_velocity = - RB_info.point.y
    LB_velocity = LB_info.point.y

    print("Velocities: ", RM_velocity, LM_velocity, RB_velocity, LB_velocity)
    # Put velocities in a list.
    velocities = [RM_velocity, LM_velocity, RB_velocity, LB_velocity]

    current_min_velocity = min(velocities)
    current_min_index = velocities.index(current_min_velocity)
    print("Current min velocity:", current_min_velocity)
    print("Current min index:", current_min_index)

    if current_min_index == 0:
        rm_pwm_value += pwm_factor
        if rm_pwm_value > max_pwm:
            rm_pwm_value = max_pwm
    elif RM_velocity < (current_min_velocity - velocity_error) or RM_velocity > (current_min_velocity + velocity_error):
        rm_pwm_value -= pwm_factor
        if rm_pwm_value < min_pwm:
            rm_pwm_value = min_pwm
    if current_min_index == 1:
        lm_pwm_value = 1
        if lm_pwm_value > max_pwm:
            lm_pwm_value = max_pwm
    elif LM_velocity < (current_min_velocity - velocity_error) or LM_velocity > (current_min_velocity + velocity_error):
        lm_pwm_value -= pwm_factor
        if lm_pwm_value < min_pwm:
            lm_pwm_value = min_pwm
    if current_min_index == 2:
        rb_pwm_value = 1
        if rb_pwm_value > max_pwm:
            rb_pwm_value = max_pwm
    elif RB_velocity < (current_min_velocity - velocity_error) or RB_velocity > (current_min_velocity + velocity_error):
        rb_pwm_value -= pwm_factor
        if rb_pwm_value < min_pwm:
            rb_pwm_value = min_pwm
    if current_min_index == 3:
        lb_pwm_value = 1
        if lb_pwm_value > max_pwm:
            lb_pwm_value = max_pwm
    elif LB_velocity < (current_min_velocity - velocity_error) or LB_velocity > (current_min_velocity + velocity_error):
        lb_pwm_value -= pwm_factor
        if lb_pwm_value < min_pwm:
            lb_pwm_value = min_pwm

    lf_pwm_value = 1
    rf_pwm_value = 0.85

    # Mega codes are not updated, so we are sending middle values to front and front values to middle. bullshit
    arr = Float32MultiArray()
    arr.data = [lm_pwm_value, lf_pwm_value, lb_pwm_value, rm_pwm_value, rf_pwm_value, rb_pwm_value]
    forward_pwm_publisher.publish(arr)


def setup():
    global forward_pwm_publisher
    # Initialize the node.
    rospy.init_node('autonomy_controller', anonymous=True)
    forward_pwm_publisher = rospy.Publisher("joyinputx", Float32MultiArray, queue_size=5)
    # Encoder data subscribers, using time synch in order to receive data simultaneously.
    right_mid_encoder_sub = message_filters.Subscriber("RM_info", PointStamped)
    left_mid_encoder_sub = message_filters.Subscriber("LM_info", PointStamped)
    right_back_encoder_sub = message_filters.Subscriber("RB_info", PointStamped)
    left_back_encoder_sub = message_filters.Subscriber("LB_info", PointStamped)
    time_synch = message_filters.ApproximateTimeSynchronizer(
        [right_mid_encoder_sub, left_mid_encoder_sub, right_back_encoder_sub, left_back_encoder_sub], 10, 0.1)
    time_synch.registerCallback(autonomy_callback)
    # Pwm publishers for every single motor.
    rospy.spin()


if __name__ == '__main__':
    setup()
