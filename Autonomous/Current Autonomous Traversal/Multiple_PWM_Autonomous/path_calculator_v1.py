#!/usr/bin/env python
import math
import numpy as np

class PathCalculator:

    def __init__(self, target_point_x, target_point_y, rover_location_x, rover_location_y, current_angle):
        self.line_angle = calculate_line_angle(target_point_x, target_point_y, rover_location_x, rover_location_y)
        self.target_angle = calculate_target_angle(self.line_angle, current_angle)
        self.target_distance = calculate_target_distance(target_point_x, target_point_y ,rover_location_x, rover_location_y)

    def calculate_line_angle(self, target_point_x, target_point_y ,rover_location_x, rover_location_y):

        relative_x = target_point_x - rover_location_x
        relative_y = target_point_y - rover_location_y

        tan = relative_y * 1.0 / relative_x

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

    def calculate_target_angle(self, line_angle, currrent_angle):
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

    def calculate_target_distance(self, target_point_x, target_point_y ,rover_location_x, rover_location_y ) :

        relative_x = target_point_x - rover_location_x
        relative_y = target_point_y - rover_location_y

        target_distance = math.sqrt(math.pow(relative_x, 2) + math.pow(relative_y, 2))
        return target_distance
