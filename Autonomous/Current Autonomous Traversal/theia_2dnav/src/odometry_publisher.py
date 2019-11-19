#!/usr/bin/env python
import rospy
import message_filters
import math
import tf
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, PointStamped

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
# Initialize previous angle.
previous_angle = 0

def odometry_callback(gyro_data, RM_info, LM_info, RB_info, LB_info):
	global RM_distance, LM_distance, RB_distance, LB_distance, current_angle, x, y, is_initial_gyro_angle_taken, previous_time

	# Take initial gyro data, calculate distance and target angle.
	if not is_initial_gyro_angle_taken:
		get_initial_gyro_angle(gyro_data)
		is_initial_gyro_angle_taken = True
		# Initialize previous time.
		previous_time = rospy.Time.now()
	else:
		global RM_distance, LM_distance, RB_distance, LB_distance, current_angle, x, y
		# Calculate distance traveled between callbacks.
		RM_distance = RM_info.point.x - RM_distance
		LM_distance = LM_info.point.x - LM_distance
		RB_distance = RB_info.point.x - RB_distance
		LB_distance = LB_info.point.x - LB_distance
		# Calculate time elapsed.
		time_duration = rospy.Time.now() - previous_time
		time_duration = time_duration.to_sec()
		# Equal previous time to ros time now.
		previous_time = rospy.Time.now()
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
		# Update x and y values.
		x += distance_center * math.cos(math.radians(current_angle))
		y += distance_center * math.sin(math.radians(current_angle))
		# Update current angle.
        	angle_difference = gyro_data.point.x - current_angle
		current_angle = gyro_data.point.x
		# Calculate velocities.
		x_velocity = distance_center * math.cos(math.radians(current_angle)) / time_duration
		y_velocity = distance_center * math.sin(math.radians(current_angle)) / time_duration
		angular_velocity = math.radians(angle_difference) / time_duration
		# since all odometry is 6DOF we'll need a quaternion created from yaw
		odom_quat = tf.transformations.quaternion_from_euler(0, 0, angle_difference)
		# first, we'll publish the transform over tf
		odom_broadcaster.sendTransform(
		    (x, y, 0.),
		    odom_quat,
		    previous_time,
		    "base_link",
		    "odom"
		)
		# next, we'll publish the odometry message over ROS
		odom = Odometry()
		odom.header.stamp = previous_time
		odom.header.frame_id = "odom"
		# set the position
		odom.pose.pose = Pose(Point(x, y, 0.), Quaternion(*odom_quat))
		# set the velocity
		odom.child_frame_id = "base_link"
		odom.twist.twist = Twist(Vector3(x_velocity, y_velocity, 0), Vector3(0, 0, angular_velocity))
		# publish the message
		odom_pub.publish(odom)
		# Equal previous time to ros time now.
		previous_time = rospy.Time.now()


def get_initial_gyro_angle(gyro_data):
	global current_angle
	current_angle = gyro_data.point.x

def setup():
	global rotate_pwm_publisher, forward_pwm_publisher, previous_time, odom_pub, odom_broadcaster
	# Initialize the node.
	rospy.init_node('odometry_publisher', anonymous = True);
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
        # Odometry and tf publishers.
        odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
        odom_broadcaster = tf.TransformBroadcaster()
	rospy.spin()


if __name__ == '__main__':
	setup()
