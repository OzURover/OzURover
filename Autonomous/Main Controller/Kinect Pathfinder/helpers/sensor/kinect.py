import math
import os

import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, PoseStamped


class Kinect:

	def __init__(self):
		self.reset(0, 0, 0)
		self.last_samples = []
		self.status_code = -1
		self.heading = 0.0

	def start(self):
		rospy.Subscriber("orb_slam2_rgbd/pose", PoseStamped,
						 self.process, queue_size=10)

		rospy.Subscriber("orb_slam2_rgbd/status", PointStamped,
						 self.process_status, queue_size=10)

		self.reset_mapper()

	def process_status(self, data):
		self.status_code = data.point.x

	def reset(self, x, y, w=0):
		self.initial_pose = (x, y, w)

	def reset_mapper(self):
		os.system(
			"rosrun dynamic_reconfigure dynparam set /orb_slam2_rgbd reset_map True")

	def status(self):
		if len(self.last_samples) < 5:
			return -1
		elif self.status_code == 3:
			return 0
		else:
			return 1

	def get_location(self):
		return self.initial_pose[0] + self.current_pose[0], self.initial_pose[1] + self.current_pose[1]

	def travel_vector(self):
		cx, cy, _ = self.current_pose
		return math.sqrt(cx**2 + cy**2)

	def set_heading(self, deg):
		self.heading = deg

	def process(self, data):
		p = data.pose.position
		o = data.pose.orientation
		if len(self.last_samples) == 5:
			self.last_samples.pop(0)
		self.last_samples.append((p.x*100.0, p.y*100.0, o.z))

		if len(self.last_samples) == 5:
			xs = np.array([x for (x, _, _) in self.last_samples])
			ys = np.array([y for (_, y, _) in self.last_samples])

			xsm, ysm = np.mean(xs, axis=0), np.mean(ys, axis=0)
			xss, yss = np.std(xs, axis=0), np.std(ys, axis=0)

			filtered = [(x, y, o) for (x, y, o) in self.last_samples if (
				x > xsm - 2 * xss) and (x < xsm + 2 * xss)]
			filtered = [(x, y, o) for (x, y, o) in filtered if (
				y > ysm - 2 * yss) and (y < ysm + 2 * yss)]

			if len(filtered) > 0:
				pose = filtered[len(filtered)-1]
				cx, cy, co = pose
				x, y = -cy, cx

				# Correct pose by rotate
				a1, a2 = 0.0, self.heading
				d = abs(a1-a2) % 360
				r = 360 - d if d > 180 else d
				sign = 1 if (a1 - a2 >= 0 and a1 - a2 <= 180) or (a1 -
																  a2 <= -180 and a1 - a2 >= -360) else -1
				r *= sign
				theta = np.radians(r)
				mat = np.array(((np.cos(theta), -np.sin(theta)),
								(np.sin(theta),  np.cos(theta))))
				v = np.array((x, y))
				r = mat.dot(v)
				self.current_pose = r[0], r[1], co
