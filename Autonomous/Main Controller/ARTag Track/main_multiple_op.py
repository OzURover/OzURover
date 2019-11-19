import rospy
import datetime
import sys
import os
import time
import math
import timeit
import numpy as np
import xml.etree.cElementTree as ET

from helpers.sensor.artag import ARTag
from helpers.sensor.lidar import LiDAR
from helpers.mapper.optimal_path import OptimalPath
from helpers.driver.RedunancyDriver import Driver
from helpers.servo.servo_controller import ServoController
from helpers.mapper.algorithms import calculate_route, calculate_position


def fine_tune(marker):
	if abs(marker[2]) > 30 or abs(marker[3]) > 30:
		return None
	print("Current read: " + str(marker[1]) +
		  " cm / " + str(marker[2]) + " deg")
	print("Fine tuning marker: " + str(marker[0]))

	hangle, vangle = SERVO.get_pos()
	SERVO.cam_hor(hangle - marker[2])
	SERVO.cam_ver(vangle + marker[3])
	time.sleep(2)

	fault = False
	err_iter_count = 0
	adj_iter_count = 0
	CEN_ANG = 0.2
	while True:
		m = AR.get_markers()
		try:
			_, _, h, v = list(filter(lambda x: x[0] == marker[0], m))[0]
			err_iter_count = 0
		except Exception:
			err_iter_count = err_iter_count + 1
			if err_iter_count > 100:
				print("Lost sight of ARTag!")
				fault = True
				break
			time.sleep(0.1)
			continue
		sys.stdout.write("Tuning... h: " + str(round(h, 2)) +
						 " v:" + str(round(v, 2)) + " iter: " + str(adj_iter_count) + "\r")
		sys.stdout.flush()
		if adj_iter_count > 20 and adj_iter_count % 8 == 0:
			CEN_ANG = CEN_ANG + 0.05
			if CEN_ANG > 1.2:
				fault = True
				break
		if abs(h) > CEN_ANG:
			inc = -1 if h > 0 else 1
			SERVO.cam_hor(inc=inc)
		if abs(v) > CEN_ANG:
			inc = -1 if v < 0 else 1
			if not SERVO.cam_ver(inc=inc):
				print("Closest degree: " + str(v))
				fault = True
				break
		if abs(h) <= CEN_ANG and abs(v) <= CEN_ANG:
			break

		adj_iter_count = adj_iter_count + 1
		time.sleep(0.4)

	print("\nCertainty: " + str((1 - CEN_ANG) * 100) + "%")

	hang, vang = SERVO.get_pos()
	if fault:
		print("Fault: Will use 20 degree error to estimate distance.")
		dist = LIDAR.get_min()
	else:
		dist = LIDAR.get_accurate(marker[1])

	# Closest one will probably be more accurate
	if dist > marker[1]:
		print("LiDAR calculation was off. Will use 20 degree error to estimate distance.")
		dist = LIDAR.get_min()
		if dist > marker[1]:
			print("LiDAR calculation was way too off. Using camera calculated distance")
			dist = marker[1]

	abs_dist = dist
	if vang > 0:
		dist = dist*math.cos(math.radians(abs(vang)))
	else:
		dist = math.sqrt(97.5**2 + dist**2 - 2*97.5*dist *
						 math.cos(math.radians(90-abs(vang))))
	print("Fine read: " + str(round(dist, 4)) +
            " cm [" + str(round(abs_dist, 4)) + " cm] / " + str(round(hang, 4)) + " deg")
	return (marker[0], dist, hang)


def get_position():
	global prev_position
	start = timeit.default_timer()
	print("Reset servos")
	SERVO.cam_hor(-150)
	SERVO.cam_ver(-5)
	time.sleep(3.5)
	print("Calibrating camera")
	AR.reset()

	TURN_DELAY = 1
	TOLERANCE_ANGLE = 20.0
	ID_MAXLIM = 10
	ID_MINLIM = -1

	current_h_deg = -150
	markers = []
	while True:
		if (current_h_deg == 160):
			break
		SERVO.cam_hor(current_h_deg)
		SERVO.cam_ver(-5)
		time.sleep(TURN_DELAY)
		m = AR.get_markers()

		print("Current degree: " + str(current_h_deg) + " found markers: " +
			  str(list(x[0] for x in markers)) + " markers in current view: " + str(list(x[0] for x in m)))

		if len(markers) > 0:
			for i in range(len(m)):
				upper = m[i]
				add = True
				for y in range(len(markers)):
					inner = markers[y]
					if upper[0] == inner[0]:
						add = False

				if abs(upper[2]) > TOLERANCE_ANGLE or upper[0] > ID_MAXLIM or upper[0] <= ID_MINLIM:
					add = False
				if add:
					marker = fine_tune(upper)
					if marker is not None:
						markers.append(marker)
		else:
			if len(m) > 0:
				for i in range(len(m)):
					if abs(m[i][2]) < TOLERANCE_ANGLE and m[i][0] < ID_MAXLIM or m[i][0] <= ID_MINLIM:
						marker = fine_tune(m[i])
						if marker is not None:
							markers.append(marker)

		current_h_deg = current_h_deg + 10

	pos = calculate_position(landmarks, markers, prev_position)
	stop = timeit.default_timer()
	print("Got position in: " + str(round(stop - start, 2)) + " seconds")
	print("Found markers: \n" + str(markers))
	return pos


def wait():
	time.sleep(1)
	while not DRIVER.is_waiting():
		time.sleep(1)
		pass


def get_current_position():
	global prev_position
	(x, y, ref) = get_position()
	if (x, y, ref) == (-1, -1, -1):
		DRIVER.new_target(0.0, 180.0)
		wait()
		(x, y, ref) = get_position()
	else:
		prev_position = (x, y)
	print("Current position: (" + str(x) + ", " + str(y) + ")")
	return (x, y, ref)


if __name__ == "__main__":
	global landmarks, prev_position
	script_dir = os.path.dirname(__file__)
	# Initialize the node.
	rospy.init_node('autonomy_controller', anonymous=True)
	AR = ARTag()
	DRIVER = Driver()
	SERVO = ServoController()
	OP = OptimalPath()
	LIDAR = LiDAR()

	AR.start()
	DRIVER.start()
	SERVO.start()
	LIDAR.start()
	OP.load_map(os.path.join(script_dir, 'data/DTM.csv'), bound=2, scale=10)

	print("All modules has started.")

	tolerance_radius = 100
	current_index = 0
	landmarks = []
	waypoints = []

	tree = ET.parse(os.path.join(script_dir, 'data/Landmarks.xml'))
	root = tree.getroot()

	for child in root:
		landmark = (child[0].text, child[1].text, child[2].text)
		landmark = tuple(float(x) for x in landmark)
		landmarks.append(landmark)

	tree = ET.parse(os.path.join(script_dir, 'data/Waypoints.xml'))
	root = tree.getroot()

	for child in root:
		waypoint = (child[0].text, child[1].text, child[2].text)
		waypoint = tuple(float(x) for x in waypoint)
		waypoints.append(waypoint)

	try:
		prev_position = (0, 0)
		while not rospy.core.is_shutdown():
			# Check if task is complete
			if current_index == len(waypoints):
				print("TASK COMPLETE")
				exit(0)

			# Get Current Position
			print("Getting current location... Previous: " + str(prev_position))
			(x, y, ref) = get_current_position()
			if (x, y, ref) == (-1, -1, -1):
				print("No ARTag was found. Aborting...")
				exit(1)

			# Calculate Angle then route to first waypoint
			waypoint = waypoints[current_index]
			print("Going to waypoint " +
				  str(current_index) + " at " + str(waypoint))
			# Create a route to waypoint
			_, route = OP.create_route((x, y), waypoint, scale=1)

			print("\nRoute: \n" + str(route) + "\n\n")

			for i in range(len(route)):
				curr = route[i]
				error_distance = math.sqrt((x - curr[1])**2 + (y - curr[2])**2)
				if error_distance < tolerance_radius / 2:
					continue
				print("Going to sub-waypoint " + str(i) +
					  " [" + str(round((i+1)/len(route), 2)*100) + "%]")
				print("Current sub-waypoint location: " + str(curr))
				DISTANCE, ANGLE = calculate_route(ref, curr, (x, y), landmarks)
				print("Driving " + str(round(DISTANCE, 2)) +
					  " cm to " + str(round(ANGLE, 2)) + " degree")
				DRIVER.new_target(DISTANCE, ANGLE)

				# Wait till rover reaches waypoint
				wait()

				# Validate correct position by getting current position again
				# If rover is in tolerance radius move onto next waypoint
				(x, y, ref) = get_current_position()
				if (x, y, ref) == (-1, -1, -1):
					print("No ARTag was found. Cannot validate position and aborting!")
					exit(1)
				else:
					while True:
						print("Validating if correct location was reached...")
						error_distance = math.sqrt(
							(x - curr[1])**2 + (y - curr[2])**2)
						if error_distance > tolerance_radius:
							print("Assumed we were at: " + str(curr) +
								  " but we were at: " + str((x, y)))
							print("Error correction needed.")
							DISTANCE, ANGLE = calculate_route(
								ref, curr, (x, y), landmarks)
							print("Driving " + str(round(DISTANCE, 2)) +
								  " cm to " + str(round(ANGLE, 2)) + " degree")
							DRIVER.new_target(DISTANCE, ANGLE)
							wait()
						else:
							break

						(x, y, ref) = get_current_position()
						if (x, y, ref) == (-1, -1, -1):
							print(
								"No ARTag was found but this is very unusual so aborting!")
							exit(1)
					print("Sub-waypoint " + str(i) +
						  " was reached. Moving to next one if available.")

			current_index = current_index + 1
			rospy.rostime.wallsleep(0.2)
	except KeyboardInterrupt:
		DRIVER.new_target(0.0, 0.0)
		print("keyboard interrupt, shutting down")
		rospy.core.signal_shutdown('keyboard interrupt')
