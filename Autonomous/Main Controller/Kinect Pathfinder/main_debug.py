# coding=utf-8
import math
import os
import sys
import time
import timeit
import xml.etree.cElementTree as ET
from random import randrange

import rospy
from helpers.driver.EmptyDriver import Driver
from helpers.mapper.algorithms import calculate_route
from helpers.mapper.optimal_path import OptimalPath
from helpers.sensor.kinect import Kinect

clock = timeit.default_timer()
def log_(msg):
	global clock
	print("[" + str(round(timeit.default_timer() - clock, 3)) + "] :: " + msg)

if __name__ == "__main__":
	script_dir = os.path.dirname(__file__)
	# Initialize the node.
	rospy.init_node('autonomy_controller', anonymous=True)
	DRIVER = Driver()
	KINECT = Kinect()
	OP = OptimalPath()

	DRIVER.start()
	KINECT.start()

	while KINECT.status() == -1:
		log_("Waiting for Kinect...")
		time.sleep(1)
	# @scale is what 1 x equal to in cms
	scale = 10.0
	OP.load_map(os.path.join(script_dir, 'data/DTM.csv'), bound=2, scale=scale)

	log_("Driver: " + str("READY" if DRIVER.is_waiting(val=True) else "ERROR"))
	log_("Kinect: " + str("READY" if KINECT.status() else "ERROR"))
	log_("OptimalPath: " + str("READY" if OP.status()
								else "ERROR... map not loaded"))

	if not OP.status() or not KINECT.status() == 1 or not DRIVER.is_waiting(val=True):
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
		(x, y) = (0, 0)
		KINECT.reset(x, y)
		while not rospy.core.is_shutdown():
			# Check if task is complete
			if current_index == len(waypoints):
				log_("TASK COMPLETE")
				exit(0)

			# Calculate Angle then route to first waypoint
			waypoint = waypoints[current_index]
			log_("Going to waypoint " +
				  str(current_index) + " at " + str(waypoint))
			# Create a route to waypoint
			start = timeit.default_timer()
			# _, route = OP.create_route((x, y), waypoint, scale=scale)
			route = [(0, 0, 50), (1, 50, 50), (2, 50, 0), (3, 0, 0)]
			#route = [(0, 0, 40), (0, 0, 80)]
			stop = timeit.default_timer()

			log_("\nRoute ["+str(round(stop-start, 2)) +
				  " sec]: \n" + str(route) + "\n\n")

			for i in range(len(route)):
				curr = route[i]
				error_distance = math.sqrt((x - curr[1])**2 + (y - curr[2])**2)
				if error_distance < tolerance_radius / 2:
					continue
				log_("Going to sub-waypoint " + str(i) +
					  " [" + str(round((i+1)/len(route), 2)*100.0) + "%]")
				log_("Current sub-waypoint location: " + str(curr))
				while abs(KINECT.travel_vector()) > 5:
					log_("Waiting for Kinect to stabilize...")
					x, y = KINECT.get_location()
					KINECT.reset_mapper()
					KINECT.reset(x, y)
					time.sleep(1)

				log_("Location -> " + str((x, y)))
				log_("DEBUG | kd, x, y, k(x, y) :: " + str([KINECT.travel_vector(), x, y, KINECT.get_location()]))
				DISTANCE, ANGLE = calculate_route((x, y), curr)
				log_("DEBUG | last sub-waypoint :: " + str(route[i-1] if i > 0 else "N/A"))
				log_("DEBUG | create_route | D, A, curr, x, y :: " + str([DISTANCE, ANGLE, curr, x, y]))
				log_("Driving " + str(round(DISTANCE, 2)) +
					  " cm to " + str(round(ANGLE, 2)) + " degree")
				DRIVER.new_target(DISTANCE, ANGLE)

				# Wait till rover reaches waypoint
				ERROR_MARGIN = 13 #0.6
				ERROR_START = 5
				fault = False
				rotation = False
				while True:
					while not DRIVER.is_waiting(KINECT.travel_vector()) or fault:
						ST = KINECT.status()
						if ST == -1:
							log_("Waiting for Kinect...")
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
							KINECT.set_heading(ANGLE)

							# Resume driver
							DRIVER.move(pause=0)
						elif ST == 0:
							DRIVER.halt()
							log_(
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
								log_("Recovered and driving " + str(round(DISTANCE, 2)) +
									  " cm to " + str(round(ANGLE, 2)) + " degree")
								DRIVER.new_target(DISTANCE, ANGLE)
								fault = False
							x, y = KINECT.get_location()
							kd = KINECT.travel_vector()
							ed = kd#DRIVER.get_travelled()

							log_("DEBUG | kd, x, y :: " + str([round(x, 2) for x in [kd, x, y]]))

							#abs(min(kd*.1, ed*.1)/max(kd*.1, ed*.1, 0.0001))
							if abs(kd-ed) > ERROR_MARGIN and ed > ERROR_START and kd > ERROR_START:
								DRIVER.halt()
								log_("Skidding detected!!!")
								log_(
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

				print("")
				log_("Sub-waypoint " + str(i+1) +
						" was reached. Resetting mapper.")
				x, y = KINECT.get_location()
				KINECT.reset_mapper()
				KINECT.reset(x, y)
				log_("Location -> " + str((x, y)))
				log_("Moving to next sub-waypoint if available.")

			current_index = current_index + 1
			time.sleep(5)  # Goal wait time
			rospy.rostime.wallsleep(0.1)
	except KeyboardInterrupt:
		log_("keyboard interrupt, shutting down")
		rospy.core.signal_shutdown('keyboard interrupt')
