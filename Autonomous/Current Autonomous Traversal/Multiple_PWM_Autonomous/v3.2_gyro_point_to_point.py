#!/usr/bin/env python
import rospy
import time
import message_filters
from autonomy_controller import AutonomyController
from point import Point
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped
#sarisin
# Initialize points.
p1 = Point(0,0)
p2 = Point(88,0)
# Set variable for initial angle.
is_initial_gyro_angle_taken = False
# Boolean for rotating or not.
is_rover_rotating = True
# Current angle.
current_angle = 90
# Distance traveled
distance_traveled = 0

def turn_callback(gyro_data, RM_info, LM_info, RB_info, LB_info):
	global distance_traveled, is_rover_rotating, RM_data_to_subtract, LM_data_to_subtract, RB_data_to_subtract, LB_data_to_subtract, LM_distance, RB_distance, LB_distance, autonomy_calculator, distance_traveled
	# Take initial gyro data.
	if not is_initial_gyro_angle_taken:
		calculate_angle_and_distance(gyro_data)
	else:
		# Angle that rover travelled so far.
        	angle_traveled = initial_gyro_angle - gyro_data.point.x
		if angle_traveled < 0:
			angle_traveled += 360
		# If angle_traveled, is in the boundaries of tolerance then start moving straight.
		if autonomy_calculator.is_target_angle_reached(angle_traveled):
			if is_rover_rotating:
				print("Waiting for 1 second. Was turning")
				arr = Float32MultiArray()
				arr.data = [0, 0, 0, 0, 0, 0]
				rotate_pwm_publisher.publish(arr)
				time.sleep(0.5)
				is_rover_rotating = False
				# Add traveled distance to rover's each wheel's previous position.
				RM_data_to_subtract = RM_info.point.x
                		LM_data_to_subtract = LM_info.point.x
                		RB_data_to_subtract = RB_info.point.x
                		LB_data_to_subtract = LB_info.point.x
			else:
				print("Going forward")
				RM_distance = abs(RM_info.point.x - RM_data_to_subtract)
				LM_distance = abs(LM_info.point.x - LM_data_to_subtract)
				RB_distance = abs(RB_info.point.x - RB_data_to_subtract)
				LB_distance = abs(LB_info.point.x - LB_data_to_subtract)
                		# Total distance traveled so far.
                		distance_traveled += (RM_distance + LM_distance + RB_distance + LB_distance) / 4
				print("Distance traveled:", distance_traveled)
                		# Equal previous data to current one for next callback.
                		RM_data_to_subtract = RM_info.point.x
                		LM_data_to_subtract = LM_info.point.x
                		RB_data_to_subtract = RB_info.point.x
                		LB_data_to_subtract = LB_info.point.x

				if distance_traveled >= target_distance:
					# Finish the task.
					arr = Float32MultiArray()
                    			arr.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
					forward_pwm_publisher.publish(arr)
					print("Target reached")
                    			rospy.signal_shutdown("Target reached.")
				else:
					# Continue to move.
					arr = Float32MultiArray()
                    			arr.data = [0.5, 0.5, 0.5, 0.335, 0.335, 0.335]
					forward_pwm_publisher.publish(arr)
		else:
			if is_rover_rotating:
				arr = Float32MultiArray()
				print("Angle traveled:", angle_traveled)
				# Turn left.
				if target_angle - angle_traveled > 0:
                    			print("Turning left")
					pwm_value = -0.4
                    			arr.data = [pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value]
                    			rotate_pwm_publisher.publish(arr)
				# Turn right.
				else:
                    			print("Turning right")
					pwm_value = 0.4
                    			arr.data = [pwm_value, pwm_value, pwm_value, pwm_value, pwm_value, pwm_value]
                    			rotate_pwm_publisher.publish(arr)
			else:
                		print("Waiting for 1 second. Was going forward")
				arr = Float32MultiArray()
				arr.data = [0, 0, 0, 0, 0, 0]
				rotate_pwm_publisher.publish(arr)
                		time.sleep(0.5)
				is_rover_rotating = True

def calculate_angle_and_distance(gyro_data):
	global initial_gyro_angle, line_angle, target_angle, target_distance, is_initial_gyro_angle_taken, autonomy_calculator
	initial_gyro_angle = gyro_data.point.x
	line_angle = autonomy_calculator.line_angle
	target_angle = autonomy_calculator.target_angle
	target_distance = autonomy_calculator.target_distance
	is_initial_gyro_angle_taken = True

def setup():
	global rotate_pwm_publisher, forward_pwm_publisher, autonomy_calculator
	# Initialize the node.
	rospy.init_node('autonomy_controller', anonymous = True);
    	# Autonomy controller class.
    	autonomy_calculator = AutonomyController(p2, p1, current_angle)
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
    	time_synch.registerCallback(turn_callback)
	rospy.spin()


if __name__ == '__main__':
	setup()
