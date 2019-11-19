#!/usr/bin/env python
import rospy
import message_filters
from path_calculator_v1 import PathCalculator
from point import Point
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped

# Initialize points.
p1 = Point(0,0)
p2 = Point(1,-1)
# Initialize point array.
point_arr = [p1,p2]
# Set variable for initial angle.
is_initial_angle_taken = False
# Boolean for rotating or not.
is_rover_rotating = True
# Setting the tolerance for the angle.
tolerance_angle = 1
# Initial gyro angle.
initial_gyro_angle = 0
# Current angle.
current_angle = 90
# Static line angle for testing.
line_angle = 0
# Static target angle for testing.
target_angle = 0
# Target distance.
target_distance = 0
# Current traveled distance (straight).
distance_traveled = 0

def turn_callback(gyro_data, RM_info, LM_info, RB_info, LB_info):
	global is_initial_angle_taken, initial_gyro_angle, line_angle, target_angle, target_distance,
	distance_traveled, is_rover_rotating, RM_data_to_subtract, LM_distance, RB_distance, LB_distance

	# Take initial gyro data.
	if not is_initial_gyro_angle_taken:
		initial_gyro_angle = gyro_data.data[0]
        path_calculator = PathCalculator(point_arr[0].x, point_arr[0].y, point_arr[1].x, point_arr[1].y, current_angle)
        line_angle = path_calculator.line_angle
        target_angle = path_calculator.target_angle
        target_distance = path_calculator.target_distance
		is_initial_gyro_angle_taken = True
	else:
		# Angle that rover travelled so far.
        angle_traveled = gyro_data.data[0] - initial_gyro_angle
		# If angle_traveled, is in the boundaries of tolerance then start moving straight.
		if (angle_traveled  < target_angle + tolerance_angle and angle_traveled > target_angle - tolerance_angle) or
		(angle_traveled < 360 + target_angle + tolerance_angle and angle_traveled > 360 + target_angle - tolerance_angle):
            if is_rover_rotating:
				print("Angle reached!")
				arr = Float32MultiArray()
				arr.data = [0, 0, 0, 0, 0, 0]
				rotate_pwm_publisher.publish(arr)
				is_rover_rotating = False
				# Add traveled distance to rover's each wheel's previous position.
				RM_data_to_subtract = RM_info.point.x
                LM_data_to_subtract = LM_info.point.x
                RB_data_to_subtract = RB_info.point.x
                LB_data_to_subtract = LB_info.point.x
			else:
				RM_distance = abs(RM_info.point.x - RM_data_to_subtract)
				LM_distance = abs(LM_info.point.x - LM_data_to_subtract)
				RB_distance = abs(RB_info.point.x - RB_data_to_subtract)
				LB_distance = abs(LB_info.point.x - LB_data_to_subtract)
                # Total distance traveled so far.
                distance_traveled += (RM_distance + LM_distance + RB_distance + LB_distance) / 4
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
			     	rospy.signal_shutdown("Target reached.")
				else:
					# Continue to move.
					arr = Float32MultiArray()
				    arr.data = [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]
					forward_pwm_publisher.publish(arr)
		else:
			if is_rover_rotating:
				print("Angle traveled:", (angle_traveled))
				arr = Float32MultiArray()
				if target_angle < 0:
					arr.data = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
					rotate_pwm_publisher.publish(arr)
				else:
					arr.data = [-0.5, -0.5, -0.5, -0.5, -0.5, -0.5]
					rotate_pwm_publisher.publish(arr)
			else:
				arr = Float32MultiArray()
				arr.data = [0, 0, 0, 0, 0, 0]
				rotate_pwm_publisher.publish(arr)
				is_rover_rotating = True

def setup():
	global rotate_pwm_publisher
	# Initialize the node.
    global forward_pwm_publisher
	rospy.init_node('autonomy_controller', anonymous = True);
	# Publisher for pwm.
	rotate_pwm_publisher = rospy.Publisher("joyinputy", Float32MultiArray, queue_size=100)
	forward_pwm_publisher = rospy.Publisher("joyinputx", Float32MultiArray, queue_size=5)
	# Gyro subscriber.
	bno_subscriber = message_filters.Subscriber("bno055_sensor", Float32MultiArray, turn_callback)
    right_mid_encoder_sub = message_filters.Subscriber("RM_info", PointStamped)
    left_mid_encoder_sub = message_filters.Subscriber("LM_info", PointStamped)
    right_back_encoder_sub = message_filters.Subscriber("RB_info", PointStamped)
    left_back_encoder_sub = message_filters.Subscriber("LB_info", PointStamped)
    time_synch = message_filters.ApproximateTimeSynchronizer(
        [bno_subscriber, right_mid_encoder_sub, left_mid_encoder_sub, right_back_encoder_sub, left_back_encoder_sub], 10, 0.1)
    time_synch.registerCallback(turn_callback)
	rospy.spin()


if __name__ == '__main__':
	setup()
