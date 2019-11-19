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
# Set variable for initial rotating.
is_initial_rotating_completed = False
# Boolean for rotating or not.
is_rover_rotating = True
# Current angle.
current_rover_angle = 90
# Slope angle variables.
current_rover_slope_angle = 0
previous_rover_slope_angle = 0
# Distance traveled
distance_traveled = 0
# Set variables for distance traveled for each encoder.
RM_distance = 0
LM_distance = 0
RB_distance = 0
LB_distance = 0
# Pwm value for right wheels.
right_pwm_value = 0.335
# Pwm value for left wheels.
left_pwm_value = 0.5

def turn_callback(gyro_data, RM_info, LM_info, RB_info, LB_info):
	global distance_traveled, is_rover_rotating, autonomy_calculator, distance_traveled, RM_distance, LM_distance, RB_distance, LB_distance

	# Take initial gyro data, calculate distance and target angle.
	if not is_initial_gyro_angle_taken:
		calculate_angle_and_distance(gyro_data)
	# Check if initial rotating is not completed or not, if not continue to rotate.
	elif not is_initial_rotating_completed:
		# Calculate angle traveled so far.
        angle_traveled = calculate_angle_traveled(gyro_data.point.x)
		# If rotating is not completed, continue to rotate.
		if not autonomy_calculator.is_target_angle_reached(angle_traveled):
			manage_initial_rotation(angle_traveled)
		# If it is completed, halt for half of a second.
		else:
			is_initial_rotating_completed = True
			halt_initial_rotation()
			# Ignore distance traveled while rotating.
			RM_distance = RM_info.point.x
			LM_distance = LM_info.point.x
			RB_distance = RB_info.point.x
			LB_distance = LB_info.point.x
	else:
		# Calculate angle traveled so far while moving.
        angle_traveled = calculate_angle_traveled(gyro_data.point.x)
		# Calculate distance traveled so far.
		distance_traveled = calculate_distance_traveled()

		# Check if we are reached to target distance or not.
		if distance_traveled >= target_distance:
			# Finish the task.
			arr = Float32MultiArray()
			arr.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
			forward_pwm_publisher.publish(arr)
			print("Target reached")
			rospy.signal_shutdown("Target reached.")
		else:
			# If we are off the target angle, then play with pwm values.
			if not autonomy_calculator.is_target_angle_reached(angle_traveled):
				# Should move to the left.
				if target_angle - angle_traveled > 0:
					right_pwm_value += 0.001
					if right_pwm_value > 1:
						right_pwm_value = 1
					left_pwm_value -= 0.001
					if left_pwm_value < 0.2:
						left_pwm_value = 0.2
				# Should move to the right.
				else:
					right_pwm_value -= 0.001
					if right_pwm_value < 0.2:
						right_pwm_value = 0.2
					left_pwm_value += 0.001
					if left_pwm_value > 1:
						left_pwm_value = 1
				# Continue to move.
				move_forward()
			else:
				# Continue to move.
				move_forward()

def calculate_angle_and_distance(gyro_data):
	global initial_gyro_angle, line_angle, target_angle, target_distance, is_initial_gyro_angle_taken, autonomy_calculator, current_rover_slope_angle
	initial_gyro_angle = gyro_data.point.x
    current_rover_slope_angle = gyro_data.point.y
	line_angle = autonomy_calculator.line_angle
	target_angle = autonomy_calculator.target_angle
	target_distance = autonomy_calculator.target_distance
	is_initial_gyro_angle_taken = True

def calculate_angle_traveled(current_gyro_angle):
	# Angle that rover travelled so far.
	angle_traveled = initial_gyro_angle - current_gyro_angle
	if angle_traveled > 180:
		angle_traveled -= 360
	elif angle_traveled < -180:
		angle_traveled += 360
	return angle_traveled

def manage_initial_rotation(angle_traveled):
	global rotate_pwm_publisher
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

def halt_initial_rotation():
	global rotate_pwm_publisher
	print("Initial rotating is completed.")
	arr = Float32MultiArray()
	arr.data = [0, 0, 0, 0, 0, 0]
	rotate_pwm_publisher.publish(arr)
	time.sleep(0.5)

def calculate_distance_traveled():
	global RM_distance, LM_distance, RB_distance, LB_distance, current_rover_slope_angle, previous_rover_slope_angle
	# Calculate distance traveled between callbacks.
	RM_distance = abs(RM_info.point.x - RM_distance)
	LM_distance = abs(LM_info.point.x - LM_distance)
	RB_distance = abs(RB_info.point.x - RB_distance)
	LB_distance = abs(LB_info.point.x - LB_distance)
	# Total distance traveled so far.
	distance_traveled += (RM_distance + LM_distance + RB_distance + LB_distance) * abs(math.cos(radians(current_rover_slope_angle - previous_rover_slope_angle))) / 4
	# Print distance information.
	print("Distance traveled:", distance_traveled)
	# Equal previous data to current one for next callback.
	RM_distance = RM_info.point.x
	LM_distance = LM_info.point.x
	RB_distance = RB_info.point.x
	LB_distance = LB_info.point.x
    # Equal current slope angle to previous one for next callback.
    previous_rover_slope_angle = current_rover_slope_angle
	# Return distance traveled.
	return distance_traveled

def move_forward():
	global forward_pwm_publisher
	arr = Float32MultiArray()
	arr.data = [left_pwm_value, left_pwm_value, left_pwm_value, right_pwm_value, right_pwm_value, right_pwm_value]
	forward_pwm_publisher.publish(arr)

def setup():
	global rotate_pwm_publisher, forward_pwm_publisher, autonomy_calculator
	# Initialize the node.
	rospy.init_node('autonomy_controller', anonymous = True);
	# Autonomy controller class.
	autonomy_calculator = AutonomyController(p2, p1, current_rover_angle)
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
