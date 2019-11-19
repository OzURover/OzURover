#!/usr/bin/env python
import rospy
import math
import message_filters
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PointStamped, Point

# Define initial angle.
initial_gyro_angle = 0
# Points.
x = [0, -1]
y = [0, 0]
# Set variable for initial angle.
is_initial_gyro_angle_taken = False
# Setting the tolerance for the angle.
tolerance_angle = 2
# Static line angle for testing.
line_angle = 0
# Static target angle for testing.
target_angle = 0
# Current slope.
current_angle = 90


def turn_callback(gyro_data):
	global initial_gyro_angle, is_initial_gyro_angle_taken

	if not is_initial_gyro_angle_taken:
		initial_gyro_angle = gyro_data.point.x
	        calculateLineAngle()
        	calculateTargetAngle()
		print("Target angle", target_angle)
		is_initial_gyro_angle_taken = True
	else:
        	angle_traveled = initial_gyro_angle - gyro_data.point.x
		if angle_traveled < 0:
			angle_traveled += 360
		if (angle_traveled  < target_angle + tolerance_angle and angle_traveled > target_angle - tolerance_angle) or (angle_traveled < 360 + target_angle + tolerance_angle and angle_traveled > 360 + target_angle - tolerance_angle):
			print("Angle reached!")
			arr = Float32MultiArray()
			arr.data = [0, 0, 0, 0, 0, 0]
			rotate_pwm_publisher.publish(arr)
			rospy.signal_shutdown("tank drive done!")
		else:
			print("Angle traveled:", angle_traveled)
			# Turn left.
			if target_angle - angle_traveled > 0:
				print("Turning left")
				arr = Float32MultiArray()
				pwm_value = -0.4
                    		arr.data = [pwm_value, -1.0, pwm_value, pwm_value, -1.0, pwm_value]
				rotate_pwm_publisher.publish(arr)
			# Turn right.
			else:
				print("Turning right")
				arr = Float32MultiArray()
				pwm_value = 0.4
                    		arr.data = [pwm_value, 1.0, pwm_value, pwm_value, 1.0, pwm_value]
				rotate_pwm_publisher.publish(arr)

def calculateLineAngle():
        global line_angle
        relative_x = x[1] - x[0]
        relative_y = y[1] - y[0]

        # Calculate tan of the relative x and y.
	if relative_x == 0:
		if relative_y > 0:
			line_angle = 90
		else:
			line_angle = -90
	else:
		tan = relative_y * 1.0 / relative_x
		# Calculate the angle
		if relative_y >= 0:
		    if relative_x >= 0:
			line_angle = abs(math.degrees(np.arctan(tan)))
		    else:
			line_angle = 180 - abs(math.degrees(np.arctan(tan)))
		else:
		    if relative_x >= 0:
			line_angle = 360 - abs(math.degrees(np.arctan(tan)))
		    else:
			line_angle = 180 + abs(math.degrees(np.arctan(tan)))

def calculateTargetAngle():
    global target_angle

    angle_difference = line_angle - current_angle

    if angle_difference >= 0 and angle_difference <= 180:
        target_angle = angle_difference
    elif angle_difference > 180 and angle_difference < 360:
        target_angle = angle_difference - 360
    elif angle_difference >= -180 and angle_difference <= 0:
        target_angle = angle_difference
    else:
        target_angle = angle_difference + 360

def setup():
	global rotate_pwm_publisher
	# Initialize the node.
	rospy.init_node('autonomy_controller', anonymous = True);
	# Gyro subscriber.
	rospy.Subscriber("bno_gyro",PointStamped, turn_callback)
	# Publisher for pwm.
	rotate_pwm_publisher = rospy.Publisher("joyinputy", Float32MultiArray, queue_size=100)
	rospy.spin()


if __name__ == '__main__':
	setup()
