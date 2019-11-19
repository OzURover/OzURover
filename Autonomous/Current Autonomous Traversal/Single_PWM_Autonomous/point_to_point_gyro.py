#!/usr/bin/env python
import rospy
import message_filters
import numpy as np
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped, Point

# Local X,Y position of the rover.
rover_location_x = 0
# Local X,Y position of the target point.
target_point_x = 50
target_point_y = 50
current_angle = 90

tolerance_angle = 2	#Setting the tolerance for the angle



def calculateDistance():
	distance = sqrt(relative_x**2 + target_point_y**2)

def calculateTargetAngle(relative_x,target_point_y):
	if y >= 0:
		if x >= 0:
			return 180*abs(np.arctan(y/x))/np.pi
		else:
			return 180 - 180*abs(np.arctan(y/x))/np.pi
	else:
		if x >= 0:
			return 180 + 180*abs(np.arctan(y/x))/np.pi
		else:
			return 360 - 180*abs(np.arctan(y/x))/np.pi

def is_target_angle():
	global  current_angle, target_angle
	return (current_angle <= target_angle or current_angle >= target_angle)



def autonomy_callback(RM_info, LM_info, RB_info, LB_info):
	global rover_location_x
    # Calculating relative x,y postiion to the target
	relative_x = target_point_x - rover_location_x


	target_angle = calculateTargetAngle(relative_x,target_point_y)
	target_point_x = calculateDistance(relative_x,target_point_y)

	if is_target_angle(target_angle):
		RM_distance = - RM_info.point.x
		LM_distance = LM_info.point.x
		RB_distance = - RB_info.point.x
		LB_distance = LB_info.point.x

		#Add mean traveled distance to rover's previous position.
		rover_location_x = (RM_distance + LM_distance + RB_distance + LB_distance) / 4
		print("Rover location:", rover_location_x)
		if relative_x <= 0:
			# Finish the task.
			forward_pwm_publisher.publish(0)
        	rospy.signal_shutdown("Target reached.")
		else:
			# Go forward.
			forward_pwm_publisher.publish(0.7)

	else:
		RM_distance = RM_info.point.x
		LM_distance = LM_info.point.x
		RB_distance = RB_info.point.x
		LB_distance = LB_info.point.x

		if is_target_angle(target_angle):
			rospy.signal_shutdown("tank drive done!")
			time.sleep(5)
		else:
			forward_pwm_publisher.publish(0.7)
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
	forward_pwm_publisher = rospy.Publisher("joyinputx", Float32, queue_size=5)
	rospy.spin()


if __name__ == '__main__':
	setup()

