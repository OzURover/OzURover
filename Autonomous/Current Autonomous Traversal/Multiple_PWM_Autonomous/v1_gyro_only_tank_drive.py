#!/usr/bin/env python
import rospy
import message_filters
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, PointStamped
from geometry_msgs.msg import PointStamped, Point

# Define initial angle.
initial_angle = 0
# Set variable for initial angle.
is_initial_angle_taken = False
# Setting the tolerance for the angle.
tolerance_angle = 1
# Static target angle for testing.
target_angle = 5


def turn_callback(gyro_data):
	global initial_angle, is_initial_angle_taken

	if not is_initial_angle_taken:
		initial_angle = gyro_data.point.x
		print("Initial angle:", initial_angle)
		is_initial_angle_taken = True
	else:
		angle_traveled = gyro_data.point.x - initial_angle
		if angle_traveled  < target_angle + tolerance_angle and angle_traveled > target_angle - tolerance_angle:
			print("Angle reached!")
			arr = Float32MultiArray()
			arr.data = [0, 0, 0, 0, 0, 0]
			rotate_pwm_publisher.publish(arr)
			rospy.signal_shutdown("tank drive done!")
		else:
			print("Angle traveled:", (angle_traveled))
			arr = Float32MultiArray()
			arr.data = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
			rotate_pwm_publisher.publish(arr)

def setup():
	global rotate_pwm_publisher
	# Initialize the node.
	rospy.init_node('autonomy_controller', anonymous = True);
	# Gyro subscriber.
	rospy.Subscriber("bno_gyro", PointStamped, turn_callback)
	# Publisher for pwm.
	rotate_pwm_publisher = rospy.Publisher("joyinputy", Float32MultiArray, queue_size=100)
	rospy.spin()


if __name__ == '__main__':
	setup()
