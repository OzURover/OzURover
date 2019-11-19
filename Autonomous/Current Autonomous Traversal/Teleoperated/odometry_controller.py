#!/usr/bin/env python
import rospy
import message_filters
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped

# Set variables for distance traveled for each encoder.
RM_distance = 0
LM_distance = 0
RB_distance = 0
LB_distance = 0
# Distance between left and right wheels
distance_baseline = 1.07
# Current angle.
current_angle = 0
# X and Y values.
x = 0
y = 0
# Set variable for initial angle.
is_initial_gyro_angle_taken = False

def odometry_callback(gyro_data, RM_info, LM_info, RB_info, LB_info):
	global RM_distance, LM_distance, RB_distance, LB_distance, current_angle, x, y, is_initial_gyro_angle_taken	

	# Take initial gyro data, calculate distance and target angle.
	if not is_initial_gyro_angle_taken:
		get_initial_gyro_angle(gyro_data)
		is_initial_gyro_angle_taken = True
	else:
		global RM_distance, LM_distance, RB_distance, LB_distance, current_angle, x, y
		# Calculate distance traveled between callbacks.
		RM_distance = RM_info.point.x - RM_distance
		LM_distance = LM_info.point.x - LM_distance
		RB_distance = RB_info.point.x - RB_distance
		LB_distance = LB_info.point.x - LB_distance
		# Distance traveled by left wheels of the robot.
		distance_left = (LM_distance + LB_distance) / 2
		# Distance traveled by right wheels of the robot.
		distance_right = (RM_distance + RB_distance) / 2
		# Equal previous data to current one for next callback.
		RM_distance = RM_info.point.x
		LM_distance = LM_info.point.x
		RB_distance = RB_info.point.x
		LB_distance = LB_info.point.x
		# Distance traveled by the center.
		distance_center = (distance_left + distance_right) / 2
		print("Distance center: ", distance_center)
		# Update x and y values.
		x += distance_center * math.cos(math.radians(current_angle))
		y += distance_center * math.sin(math.radians(current_angle))
		# Update current angle.
		current_angle = gyro_data.point.x
		print("X: ", x)
		print("Y: ", y)
		print("Current angle: ", current_angle)

def get_initial_gyro_angle(gyro_data):
	global current_angle
	current_angle = gyro_data.point.x

def setup():
	global rotate_pwm_publisher, forward_pwm_publisher, autonomy_calculator
	# Initialize the node.
	rospy.init_node('autonomy_controller', anonymous = True);
	# Publisher for pwm.
	rotate_pwm_publisher = rospy.Publisher("joyinputy", Float32MultiArray, queue_size=5)
	forward_pwm_publisher = rospy.Publisher("joyinputx", Float32MultiArray, queue_size=5)
	# Gyro subscriber.
	bno_subscriber = message_filters.Subscriber("bno_gyro", PointStamped)
	# Subscriber for wheel encoders.
        right_mid_encoder_sub = message_filters.Subscriber("RM_info", PointStamped)
    	left_mid_encoder_sub = message_filters.Subscriber("LM_info", PointStamped)
    	right_back_encoder_sub = message_filters.Subscriber("RB_info", PointStamped)
    	left_back_encoder_sub = message_filters.Subscriber("LB_info", PointStamped)
	# Use synchonizer to get 4 encoder datas and bno data simultaneously.
    	time_synch = message_filters.ApproximateTimeSynchronizer([bno_subscriber, right_mid_encoder_sub, left_mid_encoder_sub, right_back_encoder_sub, left_back_encoder_sub], 10, 0.1, allow_headerless=True)
    	time_synch.registerCallback(odometry_callback)
	rospy.spin()


if __name__ == '__main__':
	setup()
