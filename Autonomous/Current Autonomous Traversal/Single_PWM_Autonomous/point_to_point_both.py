#!/usr/bin/env python
import rospy
import message_filters
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped, Point

# Local X position of the rover.
rover_location_x = 0
# Local X position of the target point.
target_point_x = 50
# Initial trustable count of every encoder.
RM_count = 1.0
LM_count = 1.0
RB_count = 1.0
LB_count = 1.0
# Maximum velocity threshold.
max_threshold_velocity = 50


def autonomy_callback(RM_info, LM_info, RB_info, LB_info):
	global rover_location_x
    global RM_count, LM_count, RB_count, LB_count

	RM_distance = - RM_info.point.x
	LM_distance = LM_info.point.x
	RB_distance = - RB_info.point.x
	LB_distance = LB_info.point.x

	RM_velocity = - RM_info.point.y
	LM_velocity = LM_info.point.y
	RB_velocity = - RB_info.point.y
	LB_velocity = LB_info.point.y

    if max_threshold_velocity > RM_velocity:
        RM_count += 1.0
    if max_threshold_velocity > LM_velocity:
        LM_count += 1.0
    if max_threshold_velocity > RB_velocity:
        RB_count += 1.0
    if max_threshold_velocity > LB_velocity:
        LB_count += 1.0

    # Calculate total count.
    total_count = RM_count + LM_count + RB_count + LB_count

	# Add mean traveled distance to rover's previous position.
	rover_location_x = (RM_count/total_count) * RM_distance + (LM_count/total_count) * LM_distance
	+ (RB_count/total_count) * RB_distance + (LB_count/total_count) * LB_distance

	# Calculating relative x postiion to the target
	relative_x = target_point_x - rover_location_x
	print("Rover location:", rover_location_x)

	if relative_x <= 0:
		# Finish the task.
		forward_pwm_publisher.publish(0)
        rospy.signal_shutdown("Target reached.")
	else:
		# Go forward.
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
