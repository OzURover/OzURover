#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import PointStamped, Point

# Local X position of the rover.
rover_location_x = 0
# Local X position of the target point.
target_point_x = 30


def autonomy_callback(RM_info, LM_info, RB_info, LB_info):
	global rover_location_x

	RM_distance = - RM_info.point.x
	LM_distance = LM_info.point.x
	RB_distance = - RB_info.point.x
	LB_distance = LB_info.point.x

	# Add mean traveled distance to rover's previous position.
	rover_location_x = (RM_distance + LM_distance + RB_distance + LB_distance) / 4
	print("Rover location:", rover_location_x)

	# Calculating relative x postiion to the target
	relative_x = target_point_x - rover_location_x

	if relative_x <= 0:
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

def setup():
	global forward_pwm_publisher
	# Initialize the node.
	rospy.init_node('autonomy_controller', anonymous = True);
	# Encoder data subscribers, using time synch in order to receive data simultaneously.
	RM_encoder_sub = message_filters.Subscriber("RM_info", PointStamped)
	LM_encoder_sub = message_filters.Subscriber("LM_info", PointStamped)
	RB_encoder_sub = message_filters.Subscriber("RB_info", PointStamped)
	LB_encoder_sub = message_filters.Subscriber("LB_info", PointStamped)
	time_synch = message_filters.ApproximateTimeSynchronizer([RM_encoder_sub, LM_encoder_sub, RB_encoder_sub, LB_encoder_sub], 10, 0.1)
	time_synch.registerCallback(autonomy_callback)
	# Publisher for pwm.
	forward_pwm_publisher = rospy.Publisher("joyinputx", Float32MultiArray, queue_size=5)
	rospy.spin()


if __name__ == '__main__':
	setup()
