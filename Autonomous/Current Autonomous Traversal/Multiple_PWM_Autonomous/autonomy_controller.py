#!/usr/bin/env python
import message_filters
import math
import numpy as np
from path_calculator_v1 import PathCalculator
from point import Point
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PointStamped

class AutonomyController:

    def __init__(self, target_point, current_point, current_angle):
        self.tolerance_angle = 4
        self.line_angle = self.calculate_line_angle(target_point, current_point)
        self.target_angle = self.calculate_target_angle(self.line_angle, current_angle)
        self.target_distance = self.calculate_target_distance(target_point, current_point)
        print("Target angle:", self.target_angle)
        print("Target distance:", self.target_distance)

    def calculate_line_angle(self, target_point, current_point):
        # Calculate relative x and y between two points.
        relative_x = target_point.x - current_point.x
        relative_y = target_point.y - current_point.y
        # Calculate tan of the relative x and y.
	if relative_x == 0:
		if relative_y > 0:
			line_angle = 90
		else:
			line_angle = -90
		return line_angle
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
		return line_angle

    def calculate_target_angle(self, line_angle, current_angle):
        angle_difference = line_angle - current_angle

        if angle_difference >= 0 and angle_difference <= 180:
            target_angle = angle_difference
        elif angle_difference > 180 and angle_difference < 360:
            target_angle = angle_difference - 360
        elif angle_difference >= -180 and angle_difference <= 0:
            target_angle = angle_difference
        else:
            target_angle = angle_difference + 360
        return target_angle

    def calculate_target_distance(self, target_point, current_point) :

        relative_x = target_point.x - current_point.x
        relative_y = target_point.y - current_point.y

        target_distance = math.sqrt(math.pow(relative_x, 2) + math.pow(relative_y, 2))

        return target_distance

    def is_target_angle_reached(self, angle_traveled):
        return (angle_traveled  < self.target_angle + self.tolerance_angle and angle_traveled > self.target_angle - self.tolerance_angle) or (angle_traveled < 360 + self.target_angle + self.tolerance_angle and angle_traveled > 360 + self.target_angle - self.tolerance_angle)
