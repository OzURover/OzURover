# coding=utf-8
import math
import os
import sys
import time
import timeit
import xml.etree.cElementTree as ET

import rospy
from helpers.driver.RedunancyDriver import Driver
from helpers.mapper.algorithms import calculate_route
from helpers.mapper.optimal_path import OptimalPath
from helpers.sensor.kinect import Kinect

if __name__ == "__main__":
	script_dir = os.path.dirname(__file__)
	# Initialize the node.
	rospy.init_node('autonomy_controller', anonymous=True)
	DRIVER = Driver()
	KINECT = Kinect()
	OP = OptimalPath()

	DRIVER.start()
	KINECT.start()

	# @scale is what 1 x equal to in cms
	scale = 10.0
	OP.load_map(os.path.join(script_dir, 'data/DTM.csv'), bound=2, scale=scale)

	print("Driver: " + str("READY" if DRIVER.is_waiting() else "ERROR"))
	print("Kinect: " + str("READY" if KINECT.status() else "ERROR"))
	print("OptimalPath: " + str("READY" if OP.status()
								else "ERROR... map not loaded"))

	if not OP.status() or not KINECT.status() == 1 or not DRIVER.is_waiting():
		exit(2)

	tolerance_radius = 50
	current_index = 0
	waypoints = []

	tree = ET.parse(os.path.join(script_dir, 'data/Waypoints.xml'))
	root = tree.getroot()

	for child in root:
		waypoint = (child[0].text, child[1].text, child[2].text)
		waypoint = tuple(float(x)*scale for x in waypoint)
		waypoints.append(waypoint)

	try:
		(x, y) = (3355.65, 3018.90)
		KINECT.reset(x, y)
		while not rospy.core.is_shutdown():
			# Check if task is complete
			if current_index == len(waypoints):
				print("TASK COMPLETE")
				exit(0)

			# Calculate Angle then route to first waypoint
			waypoint = waypoints[current_index]
			print("Going to waypoint " +
				  str(current_index) + " at " + str(waypoint))
			# Create a route to waypoint
			start = timeit.default_timer()
			# _, route = OP.create_route((x, y), waypoint, scale=scale)
			route = [(0, 3355.65, 3018.90), 
			(1, 2400.00, 3400.00), 
			(2, 1900.00, 3300.00), 
			(3, 1805.55, 2959.85),
			(4, 1800.00, 2400.00),
			(5, 1160.73, 2094.47),
			(6, 300.00, 2100.00),
			(7, 160.00, 1347.64),
			(8, 281.73, 820.01),
			(9, 720.00, 820.01),
			(10, 1222.89, 610.14),
			(11, 1000.00, 1077.30),
			(12, 1741.72, 1077.30)]
			#route = [(0, 0, 40), (0, 0, 80)]
			stop = timeit.default_timer()

			print("\nRoute ["+str(round(stop-start, 2)) +
				  " sec]: \n" + str(route) + "\n\n")

			for i in range(len(route)):
				curr = route[i]
				error_distance = math.sqrt((x - curr[1])**2 + (y - curr[2])**2)
				if error_distance < tolerance_radius:
					continue
				print("Going to sub-waypoint " + str(i) +
					  " [" + str(round((i+1)/len(route), 2)*100) + "%]")
				print("Current sub-waypoint location: " + str(curr))
				while abs(KINECT.travel_vector()) > 5:
					print("Waiting for Kinect to stabilize...")
					x, y = KINECT.get_location()
					KINECT.reset_mapper()
					KINECT.reset(x, y)
					time.sleep(1)

				print("Location -> " + str((x, y)))
				DISTANCE, ANGLE = calculate_route((x, y), curr)
				print("Driving " + str(round(DISTANCE, 2)) +
					  " cm to " + str(round(ANGLE, 2)) + " degree")
				DRIVER.new_target(DISTANCE, ANGLE)

				# Wait till rover reaches waypoint
				ERROR_MARGIN = 13  # 0.6
				ERROR_START = 5
				fault = False
				rotation = False
				while True:
					while not DRIVER.is_waiting() or fault:
						ST = KINECT.status()
						if ST == -1:
							print("Waiting for Kinect...")
							continue
						elif not DRIVER.is_target_angle_reached():
							if not rotation:
								x, y = KINECT.get_location()
								rotation = True
						elif DRIVER.is_target_angle_reached() and rotation:
							# Pause driver
							DRIVER.move(pause=1)
							KINECT.reset_mapper()
							KINECT.reset(x, y)
							rotation = False
							KINECT.set_heading(DRIVER.get_angle())

							# Resume driver
							DRIVER.move(pause=0)
						elif ST == 0:
							DRIVER.halt()
							print(
								"Lost track of map. Restarting from last waypoint after 8 second wait")
							x, y = KINECT.get_location()
							KINECT.reset_mapper()
							KINECT.reset(x, y)
							fault = True
							time.sleep(8)
							break
						else:
							if fault:
								# Recovering
								DISTANCE, ANGLE = calculate_route((x, y), curr)
								print("Recovered and driving " + str(round(DISTANCE, 2)) +
									  " cm to " + str(round(ANGLE, 2)) + " degree")
								DRIVER.new_target(DISTANCE, ANGLE)
								fault = False
							x, y = KINECT.get_location()
							kd = KINECT.travel_vector()
							ed = DRIVER.get_travelled()

							sys.stdout.write("Travelled: " + str(round(kd, 2)) + " / " +
											 str(round(ed, 2)) + " *- " + str(round(abs(kd-ed), 2)) + " Location -> " + str((round(x, 2), round(y, 2))) + "             /\r")
							sys.stdout.flush()

							#abs(min(kd*.1, ed*.1)/max(kd*.1, ed*.1, 0.0001))
							if abs(kd-ed) > ERROR_MARGIN and ed > ERROR_START and kd > ERROR_START:
								DRIVER.halt()
								print("Skidding detected!!!")
								print(
									"Re-calculating position and trying again in 8 seconds")
								x, y = KINECT.get_location()
								KINECT.reset_mapper()
								KINECT.reset(x, y)
								fault = True
								time.sleep(8)
								break

							time.sleep(0.05)  # REMOVE THIS
					if not fault:
						break

				print("Sub-waypoint " + str(i+1) +
					  " was reached. Resetting mapper.")
				x, y = KINECT.get_location()
				KINECT.reset_mapper()
				KINECT.reset(x, y)
				print("Location -> " + str((x, y)))
				print("Moving to next sub-waypoint if available.")

			current_index = current_index + 1
			time.sleep(5)  # Goal wait time
			rospy.rostime.wallsleep(0.1)
	except KeyboardInterrupt:
		print("keyboard interrupt, shutting down")
		rospy.core.signal_shutdown('keyboard interrupt')
