#!/usr/bin/env python
import rospy
import time
import message_filters
from autonomy_controller import AutonomyController
from point import Point
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped

#sarisin KIVIRCIIIIIIIK
# Initialize points.
p1 = Point(0,0)
p2 = Point(88,0)
# Set variable for initial angle.

is_initial_gyro_angle_taken = False

is_partial_move_started = False
is_target_headed = False


# Set variable for initial rotating.
is_initial_rotating_completed = False
# Boolean for rotating or not.
is_rover_rotating = True
# Current angle.
current_rover_angle = 90
# Distance traveled
distance_traveled = 0

def turn_callback(gyro_data, RM_info, LM_info, RB_info, LB_info):
	global distance_traveled, partial_distance_traveled, is_rover_rotating, LM_distance, RB_distance, LB_distance, autonomy_calculator, deflection_angle
	# Take initial gyro data.
	
	if not is_initial_gyro_angle_taken: # calibration process.
		calculate_angle_and_distance(gyro_data)
	
	elif not is_initial_rotating_completed: # just do it?
        # Angle that rover travelled so far.
        angle_traveled = initial_gyro_angle - gyro_data.point.x
		
	elif autonomy_calculator.is_rover_reached_checkpoint(p1,p2): # p1 ı current point olarak varsaydım.	
	
		if not autonomy_calculator.is_target_angle_reached(angle_traveled):
			print("Angle traveled:", angle_traveled)
			# Turn left.
			if target_angle - angle_traveled > 0:
				print("Turning left")
				rover_auto_driver.rotate_left()
			else:
				print("Turning right")
				rover_auto_driver.rotate_right()
		else:
				print("Initial rotating is completed.")
				rover_auto_driver.halt()
				# Add traveled distance to rover's each wheel's previous position.
				is_initial_rotating_completed = True
				time.sleep(0.5)
		else:
			# Angle that rover travelled so far.
			angle_traveled = initial_gyro_angle - gyro_data.point.x
			if angle_traveled < 0:
				angle_traveled += 360
			
			if is_rover_rotating:
				print("Waiting for 1 second. Was turning")
				rover_auto_driver.halt()
				time.sleep(0.5)
				is_rover_rotating = False

			else:
				print("Going forward")
				calculate_distance_by_mean()
				
				if distance_traveled >= target_distance: # büyük hedef tamamdır. aracı durdur.
					# Finish the task.
					rover_auto_driver.halt()
					print("Target reached")
					rospy.signal_shutdown("Target reached.")
				else:
					# Continue to move.# ileri gidiyoruz. bu kısıma ufak bir rotate-drive algoritması lazım.
					
					pwm_rotate_value = (1/smooth_tolerance_angle) * (target_angle - line_angle) # 
					rover_auto_driver.smooth_forward(target_distance, pwm_rotate_value)
					
					
def calculate_angle_and_distance(gyro_data):
	global initial_gyro_angle, line_angle, target_angle, target_distance, is_initial_gyro_angle_taken, autonomy_calculator
	initial_gyro_angle = gyro_data.point.x
	line_angle = autonomy_calculator.line_angle
	target_angle = autonomy_calculator.target_angle
	target_distance = autonomy_calculator.target_distance
	is_initial_gyro_angle_taken = True

def setup():
	global rover_auto_driver, autonomy_calculator
	# Initialize the node. 
	rospy.init_node('autonomy_controller', anonymous = True);
    	# Autonomy controller class.
    autonomy_calculator = AutonomyController(p2, p1, current_rover_angle)
	rover_auto_driver = AutonomousDriveController()
	# Publisher for pwm.
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

def calculate_distance_by_mean():
	RM_distance = abs(RM_info.point.x - RM_distance)
	LM_distance = abs(LM_info.point.x - LM_distance)
	RB_distance = abs(RB_info.point.x - RB_distance)
	LB_distance = abs(LB_info.point.x - LB_distance)
    # Total distance traveled so far.
	# !! sapma durumu için encoder değerini cos ile çarptım. 
    distance_traveled += (RM_distance + LM_distance + RB_distance + LB_distance)* math.cos(radians(target_angle - line_angle)) / 4  
	print("Distance traveled:", distance_traveled)
    # Equal previous data to current one for next callback.


			
if __name__ == '__main__':
	setup()